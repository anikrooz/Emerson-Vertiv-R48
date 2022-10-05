This project contains a module that can be included in other projects, written for ESP32 / 8266 but likely to work on other Arduino compatible hardware.
Uses the mcp_can library for CAN comms
And TelnetStream for feedback messages

For a more standalone example of controlling the PSU, see the standalone folder.

Example usage:
```
#include <WiFi.h>
#include <TelnetStream.h>
#include <mcp_can.h>
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
