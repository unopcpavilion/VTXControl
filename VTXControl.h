#include <Arduino.h>
#include "SoftwareSerialWithHalfDuplex.h"
#include "VTX_SmartAudio.h"
#include "VTX_Tramp.h"
#ifndef VTXControl_h
#define VTXControl_h
//--------------------------
//#define VTXCDEBUG 1 //Uncomment this define to see the diagnostics
//--------------------------
#if VTXCDEBUG
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif
#if VTXCDEBUG
static void dumpBuffer(const uint8_t* data, const int8_t len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.print((uint8_t)data[i] < 0x10 ? " 0" : " ");
    Serial.print((uint8_t)data[i], HEX);
  }
  Serial.println("");
}
#endif

enum VTXMode
{
  SmartAudio = 1,
  Tramp = 2,
};
enum VTXErrors
{
  vtxNoErrors = 0,  
  vtxParseResponseInvalidCRCOrBuffer = 0x01,
  vtxIncomingBytesZero = 0x02,
  vtxIncomingBytesLessHeaderSize = 0x04,
  vtxIncomingByteNotEqualSyncByte = 0x08,
  vtxIncomingByteNotEqualHeaderByte = 0x10,
  vtxPacketSizeGreaterMaxPacketSize = 0x20,
  vtxBufferLengthLessWholePacket = 0x40,
  //--
  vtxportIsNotListening = 0x0100,
  vtxportBufferIsEmpty = 0x0200,
  vtxportTxDelayIsZero = 0x0400,
  vtxportRXDelayStopBitNotSet = 0x8000,
  vtxtrampNotInited = 0x10000,
  vtxLastByteNotSyncStop = 0x20000,
};


class VTXControl
{
   
public:  
  VTXControl(
    int vtxMode, 
    int softPin,
    int responseTimeOut = 1000,
    bool smartBaudRate = true,
    int numtries = 3);
  void flush();
  void waitForInMs(unsigned int ms);
  bool setPitMode(bool enabled);//SmartAudio: v1 not supported, v2 can only leave pitmode, v2.1 full support
  bool setFrequency(uint16_t freq);//set frequency by freq value in MHz
  bool sendFrequency(uint16_t freq);//fire and forget: sends set frequency command without waiting for confirmation from VTX
  bool setPowerMw(uint16_t pwrmW);//sets power in mW (SmartAudio: converted to the nearest level supported by protocol version)
  bool setPowerDbm(uint8_t pwrDbm);//sets power in dBm (Tramp: converted to mW)
  bool setPowerLevel(uint8_t level);//SmartAudio only: sets raw power level (v1/v2: level index as VTX knows it, v2.1: index in dBm list reported by VTX)
  bool updateParameters();
  uint16_t getFrequency() { return cur_freq; }//in MHz, 0 means not updated
  uint16_t getPowerMw() { return pwr_mW; }//0 means not updated/unknown
  uint8_t getPowerDbm() { return pwr_dBm; }//0 means not updated/unknown
  int getPowerLevel() { return pwr_Level; }//SmartAudio only: raw power level, -1 means not updated/unknown
  bool getPitMode() { return pitMode; }
  static uint8_t mwToDbm(uint16_t pwrmW);
  static uint16_t dbmToMw(uint8_t pwrDbm);
  bool sa_readResponse();
  void clearErrors();
  VTXErrors getErrors();
  long getSpeed();
#if VTXCDEBUG
  bool testSMAWrite();
  bool testSMAResponseFromSerial1();
#endif
private:
  SoftwareSerialWithHalfDuplex* port;
  int vtx_mode = VTXMode::SmartAudio;//default
  VTXErrors errors = VTXErrors::vtxNoErrors;
  long sa_offerNewSpeed(long currentSpeed);//tries to offer other baud rate to work with vtx
  void setError(VTXErrors error);
  ProtocolVersion sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v2;//smart audio protocol version
  bool sa_version_known = false;//true after first successful settings response from VTX
  uint8_t sa_operation_mode = 0;//raw operation mode from last settings response
  int sa_pit_prev_level = -1;//v2.1 only: power level before entering pitmode
  uint8_t sa_dbm_levels[8];//v2.1 only: power levels in dBm reported by VTX
  uint8_t sa_dbm_levels_count = 0;
  int pwr_Level = -1;//SmartAudio raw power level, -1 means not updated (not requested from VTX)
  uint16_t pwr_mW = 0;//current power in mW, 0 means not updated
  uint8_t pwr_dBm = 0;//current power in dBm, 0 means not updated
  uint16_t cur_freq = 0;//last frequency (in MHz) reported by the VTX, 0 means not updated
  bool pitMode = false;
  bool initialized = false;//tramp protocol needs to be initialized, so this var reflects state of Tramp initialization
  int _responseTimeOut = 1000;//in ms
  int _numtries = 3;//num tries to send request and receive response, after that we try to change baud rate and try again
  bool _smartBaudRate = true;//tries to find apporpriated baud rate to communicate with VTX, if false- just work on fixed initial baudrate

  //some utility functions
  int sa_levelsCount();
  uint8_t sa_levelToDbm(int level);
  int sa_levelFromDbm(uint8_t pwrDbm);//nearest supported level
  int sa_levelFromV1(uint8_t pwrValue);
  void sa_setCurrentLevel(int level);
  void setCurrentMw(uint16_t pwrmW);
  void setCurrentDbm(uint8_t pwrDbm);
  bool trampSetPowerChecked(uint16_t pwrmW);

  bool sa_updateSettings(); 
  bool sa_ignoreCrc() const { return SMARTAUDIO_IGNORE_CRC; }  
  bool sa_parseResponseBuffer(const uint8_t* buffer);
  // command functions
  bool sa_setPitMode(bool enabled);
  bool sa_sendU8Command(uint8_t cmd, uint8_t value);
  bool sa_getSettings();
  bool sa_setFrequency(uint16_t freq);
  bool sa_setPower(uint8_t level);
  bool trampSendPacket(uint8_t* packet, bool respRequired);//trampFrame_t* packet);
  bool trampPush(const uint8_t* packet);//const trampFrame_t* object);
  bool trampSendCmd(uint8_t cmd);
  bool trampSendCmd(uint8_t cmd, uint16_t param);

  //uint8_t trampGetState();
  bool trampSetFrequency(uint16_t freq);
  bool trampSetPower(uint16_t milliWatts);
  bool trampInit();
  bool trampSetPitMode(bool enabled);
  bool trampUpdate();
  long trampOfferNewSpeed(long currentSpeed);//tries to offer other baud rate to work with vtx
  bool trampGetStatus();  
  bool trampReadResponse();    
};
#endif
