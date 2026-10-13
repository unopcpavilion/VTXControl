# VTXControl
Arduino library providing video transmitter (VTX) control by SmartAudio/Tramp protocol.

This C/C++ code uses modified SoftwareSerialWithHalfDuplex library (part code taken from CustomSoftwareSerial to support different configuration of serial port (especially 8N2)), some code taken from BetaFlight and ArduPilot (SmartAudio and Tramp protocols support) code.

This code created to use features of Tramp/SmartAudio on VTX (like switching power modes and channels/frequencies) by code/wire on Arduino driven systems or robots.

VTXControl works in two modes/protocols - SmartAudio and Tramp, communications with VTX established by software serial port (SoftwareSerialWithHalfDuplex).

This code had been tested on Eachine TX5258 (SmartAudio v2 protocol) and JHEMCU RuiBet Tran3016W (Tramp protocol) VTXes, so that's not an universal solution for your own VTX (just because your VTX can have manufacturer's own Tramp/SmartAudio protocol implementation).

PitMode: `setPitMode(bool)` / `getPitMode()`. Tramp - full support. SmartAudio v1 - not supported, v2 - only leaving pitmode (protocol can't enable it), v2.1 - enabled as 0dBm power (VTX won't boot into pitmode), power is restored when leaving.

No tables of powers/frequencies are needed: frequency is set in MHz and power in mW or dBm.

- `setFrequency(uint16_t mhz)` / `getFrequency()`
- `setPowerMw(uint16_t mW)` / `getPowerMw()` - Tramp sends mW as is; SmartAudio converts to the nearest level supported by the protocol version
- `setPowerDbm(uint8_t dBm)` / `getPowerDbm()` - Tramp converts dBm to mW
- `setPowerLevel(uint8_t level)` / `getPowerLevel()` - SmartAudio only, raw level: v1/v2 - level index as VTX knows it, v2.1 - index in list of dBm levels reported by VTX
- `VTXControl::mwToDbm(mW)` / `VTXControl::dbmToMw(dBm)` - conversion helpers

SmartAudio v1/v2 don't report power in mW, so `setPowerMw`/`getPowerMw` assume the 4 standard TBS levels (25/200/500/800 mW). For v1/v2 VTXs with their own levels (e.g. 1W..10W) use `setPowerLevel`.

A simplest creation of VTXControl instance example:

//for SmartAudio mode

VTXControl* vtx;

vtx = new VTXControl(VTXMode::SmartAudio, 53, 500, false);

or

//for Tramp mode

VTXControl* vtx;

vtx = new VTXControl(VTXMode::Tramp, 53, 500, false);

Sorry there is no support or help with this code, use it on your own.

NB: The main problem of interacting VTXControl with different VTXs is in implementation difference of SmartAudio/Tramp protocols. In other words, as BetaFlight/ArduPilot authors said, "because of poor implementation" of SmartAudio/Tramp by VTX manufacturers. So please use VTX_Test project to diagnose your own VTX - send commands, analyze responses, try different baudrates, try to add zero or not bytes to the end/start of standard frames/packets, ignore or not crc to achieve correct results with your own VTX.

**NB1: To VTX owners with >800mW power, SmartAudio v1 or v2 protocol: you can use following lifehack to find power levels of your VTX: just set different levels of power on your VTX by button and call updateParameters by VTX_Test project to get the level (getPowerLevel) for the current level of power.**
