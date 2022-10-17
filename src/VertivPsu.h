/*
  VertivPSU.h - Library for controlling Emerzon / Vertiv R48-3000 Power Supply Unit.
  Created by Ant Nikrooz, July 2022
  Released into the public domain.
*/
#ifndef VertivPSU_h
#define VertivPSU_h

#include "Arduino.h"

class VertivPSU
{
public:
  VertivPSU(int);
  void init(int);
  void setV(int vOut);
  void setCPerc(int iOutP);
  void setCAmps(float iOutA);
  void switchACpower(bool on);
  void setCanDebug(bool on);
  void setDebug(bool on);
  
  void tick();
  void bob();
  bool isConnected();
  float voutput = 0;
  float ioutput = 0;
  MCP_CAN *can0;

private:
  void updateSettings();
  void updateVoltage();
  void updateAmps();
  void sendCurrentPerc(long unsigned int);
  void sendControl();
  void sendSync();
  void sendSync2();
  void gimme5();
  void sendcommand(long unsigned int&, uint8_t*);
  void candecode();
  void hex2bin(uint8_t *out, const char *in, size_t *size);
  void hex82bin(long unsigned int&, const char*);

  
  int PIN_INTERRUPT;
  int _cs;
  bool _talking = 1;
  boolean _initialising = 1;
  boolean _got5B = 0;
  boolean _sentBothSyncs = 0;
  long unsigned int _rxId;
  unsigned char _len = 0;
  unsigned char _rxBuf[8];
  unsigned char _txBuf[8];
  long unsigned int _txId;
  char _msgString[128]; // Array to store serial string
  unsigned long _looptime = 20000;
  unsigned long _lastreceived = 0;

  int _vout = 492;  // v*10
  int _ioutp = 50;  // percent
  float _iouta = 1; // amps. Which takes precedence??
  boolean _dcOff = 0;
  boolean _fanFull = 0;
  boolean _flashLed = 0;
  boolean _acOff = 1;

  bool _debug = 0;
  bool _candebug = 0;
};

#endif
