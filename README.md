This project contains a module that can be included in other projects, written for ESP32 / 8266 but likely to work on other Arduino compatible hardware.
Painstakingly reverse engineered from CAN commands from the Emerson M820B controller
Uses the mcp_can library for CAN comms
And TelnetStream for feedback messages

For a more standalone example of controlling the PSU, see the standalone folder.

Example usage:
```
#include <WiFi.h>
#include <TelnetStream.h>
#include <mcp_can.h>
#include <SPI.h>
#include "VertivPsu.h"

#define CAN0_CS 5
VertivPSU charger(CAN0_CS);

void setup() {
  Serial.begin(115200);
  Serial.println("alive");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //set these
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    //ESP.restart();
  }

  
  TelnetStream.begin();
  Serial.println("init Can0");
  charger.init(CAN0_INT);
}

void readTelnet();

void loop() {
  charger.tick();
  static unsigned long next_measure = millis()+4000;
  if(millis() > next_measure && charger.isConnected()){
    next_measure = millis() + 1000;
    TelnetStream.printlnString((charger.ioutput));
    
  }
  
  //charger.switchACpower(1);
  //maybe read something from TelnetStream, or MQTT...
  //charger.setCAmps(1);
    // the PSU will not output less than 5A
   
   
  readTelnet();
  
}

void readTelnet(){
  switch (TelnetStream.read()) {
    case 'R':
      TelnetStream.println("eboot!");
      TelnetStream.stop();
      delay(100);
      ESP.restart();
      break;
   case 'd':
      charger.setDebug(1);
      break;
    case 'C':
      charger.setCanDebug(1);
      break; 

  }
}
```

*Note: The charger/PSU will boot up in AC ON mode, outputting its default voltage and the last set current*
To set the PSU's default current, send a frame using chargerManager.cpp over serial, like 0607FF83  03 F0 00 24 42 40 00 00 (48v, that's the minimum). Will add a function for that soon

Methods:
```
  VertivPSU(int chipSelect_Pin); //chipSelect Pin from MCP2515
  void init(int interrupt_pin);  //interrupt_pin on MCP2515 NOT CURRENTLY USED
  void setV(int vOut); //set output voltage ("float voltage" 41.5v - 58.5v in 0.1v increments)
  void setCPerc(int iOutP); //set % output current. 0% - 121% in 1% increments (I know....)
  void setCAmps(float iOutA); //set output in Amps (1A - 50A in 0.1A increments. Mine seems to ignore under 5A, may be voltage dependent)
  void switchACpower(bool on); //turn AC relay on and off
  void setCanDebug(bool on); //CAN level debug output
  void setDebug(bool on); //debug output
  
  void tick(); //add this each loop()
  bool isConnected(); //CAN comms established and charger turned off to start with?
  float voutput; //voltage right now as measured byt the PSU
  float ioutput; //current output right now
```
