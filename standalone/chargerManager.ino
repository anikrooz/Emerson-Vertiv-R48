#include <mcp_can.h>
#include <SPI.h>

boolean talking = 1;

boolean initialising = 1;
boolean got5B = 0;
boolean sentBothSyncs = 0;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
unsigned char txBuf[8];
long unsigned int txId;
char msgString[128];                        // Array to store serial string
unsigned long looptime = 0;
unsigned long lastreceived = 0;

int vout = 520; //v*10
int ioutp = 20; //percent
float iouta = 1; //amps. Which takes precedence??
boolean dcOff = 0;
boolean fanFull = 0;
boolean flashLed = 0;
boolean acOff = 1;

//#define CAN0_INT 2                              // Set INT to pin 2
//MCP_CAN CAN0(10);                               // Set CS to pin 9 depending on shield used

#define CAN0_INT 5                             // Set INT to pin 2
MCP_CAN CAN0(11);                               // Set CS to pin 9 depending on shield used



float voutput = 0;


int debug = 0;


/****
 * 
 * Todo: add timer to go back to initialising if no data for a while
 * 
 * 
 */

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  Serial.println("Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8");
  sendSync();

}

void loop()
{
  checkSerial();

  if (CAN0.checkReceive() == 3)                        // If CAN0_INT pin is low, read receive buffer
  {
    candecode();
  }

  //reset if no comms
  if (millis() > lastreceived + 20000) acOff = 1;
  if (millis() > lastreceived + 25000) initialising = 1;
  //raise an alarm!?
   


  if (millis() > looptime + (initialising ? 10000 : 6000))
  {
    looptime = millis();
    if(initialising){
      sendSync();
      gimme5(); 
    }else{
      updateSettings();
    }
  }
}



///// End LOOP

void checkSerial(){
    static char buffer[128];
    static size_t length = 0;

    

    if (Serial.available()) {
        char c = Serial.read();

        if(c=='/'){
          length=0;
          acOff = !acOff;
        }

        //on : process the ID
        else if (c == ':') {
            // Properly terminate the string
            buffer[length] = '\0';
            txId = 0;
            hex82bin(txId, buffer);
            length = 0;
        }

        // On carriage return, process the received data.
        else if (c == '\r') {
            buffer[length] = '\0';

            // Convert the hex data to a byte array.
            size_t byte_count = length/2;
            uint8_t data[byte_count];
            hex2bin(data, buffer, &byte_count);

            sendcommand(txId, data);
            // Reset the buffer for next line.
            length = 0;
        }

        // Otherwise buffer the incoming byte.
        else if (length < sizeof buffer - 1) {
            buffer[length++] = c;
        }
    }
  
}


void updateSettings(){

   //ask for info
   txId = 0x0647FF83;
   uint8_t ask[8] = { 0x03, 0xF0, 0x00, 0x08, 0x3F, 0x94, 0x62, 0x9B};
   sendcommand(txId, ask);

  
    //calc & send iout Amps
   long unsigned int currh = 0x3f000000;
   int multiplier = 0x80;
   int dones = 0;

   for (byte ampsect = 2; ampsect < 50; ampsect *= 2){
      if(_min(iouta, ampsect) > dones){
        float ibit = _min(iouta, ampsect) - dones ;
        long unsigned int mbit = ibit*0x10000*multiplier;
        currh += mbit;
        /*
        Serial.println();
        Serial.print("left: ");
        Serial.println(ibit);
        Serial.print("    x ");
        Serial.println(multiplier, HEX);
        Serial.print("    = ");
        Serial.println(mbit, HEX);
        Serial.print("  TOT ");
        Serial.println(currh, HEX);
        */
        
        dones = ampsect;
      }
      multiplier /=2;
   }
   
   uint8_t mesg[8] = {0x03, 0xF0, 0x00, 0x1A, 0,0,0,0};
   mesg[4] = (currh & 0xff000000) >> 24;
   mesg[5] = (currh & 0x00ff0000) >> 16;
   mesg[6] = (currh & 0x0000ff00) >> 8;
   mesg[7] = currh & 0x000000ff;
   txId = 0x0607FF83;
   sendcommand(txId, mesg);


   //voltage...
   updateVoltage();

   sendCurrentPerc(0x0607FF83);
   sendCurrentPerc(0x0677FF83);

   sendControl();

}



void updateVoltage(){
   txId = 0x0607FF83;
   uint8_t mesg[8] = {0x03, 0xF0, 0x00, 0x21, 0,0,0,0};
   long unsigned int volth = (vout - 320) * 0x40000 / 10;
   mesg[4] = 0x42; //(volth & 0xff000000) >> 24;
   mesg[5] = (volth & 0x00ff0000) >> 16;
   mesg[6] = (volth & 0x0000ff00) >> 8;
   mesg[7] = volth & 0x000000ff;
   sendcommand(txId, mesg);
}

void sendCurrentPerc(long unsigned int id){
     //curr %
   uint8_t mesg[8] = {0x03, 0xF0, 0x00, 0x22, 0,0,0,0};

   long unsigned int perch = (ioutp / 10 * 0x800000) + 0x3D000000;
   mesg[4] = (perch & 0xff000000) >> 24;
   mesg[5] = (perch & 0x00ff0000) >> 16;
   mesg[6] = (perch & 0x0000ff00) >> 8;
   mesg[7] = perch & 0x000000ff;
   sendcommand(id, mesg);

}

void sendControl(){
   //control bits...
   uint8_t msg[8] = {0, 0xF0, 0, 0x80, 0, 0, 0, 0};
   msg[2] = dcOff << 7 | fanFull << 4 | flashLed <<3 | acOff << 2 | 1;
   txId = 0x06080783;
   sendcommand(txId, msg);

   looptime = millis();
}

void sendSync(){
  txId = 0x0707FF83;
  uint8_t msg[8] = {0x04, 0xF0, 0x01, 0x5A, 00, 00, 00, 00};
  sendcommand(txId, msg);
}
void sendSync2(){
  txId = 0x0717FF83;
  uint8_t msg[8] = {0x04, 0xF0, 0x5A, 00, 00, 00, 00, 00};
  sendcommand(txId, msg);

}

void gimme5(){
  txId = 0x06080783;
  uint8_t msg[8] = {0x20, 0xF0, 00, 0x80, 00, 00, 00, 00};
  sendcommand(txId, msg);
}


void sendcommand(long unsigned int &id, uint8_t tx[8])
{
  if(!talking) return;
  sprintf(msgString, "%.8lX:", id);
   Serial.print(msgString);
   for (byte i = 0; i < 8; i++) {
        sprintf(msgString, " %.2X", tx[i]);
        Serial.print(msgString);
   }
  Serial.println(" (sent)");
  //set the first bit back to 1 for extended
  CAN0.sendMsgBuf(id, 1, 8, tx); // | 0x80000000, 0, 8, tx);

}



void candecode()
{
  CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
  //if(rxBuf[0]==0x12) Serial.println("12");
  if (rxBuf[0]==0x11 ){
      //status
      lastreceived = millis();
      voutput = (float)(uint16_t(rxBuf[1] << 8) + uint16_t(rxBuf[2])) / 1022;
      Serial.print(" ");
      Serial.print(voutput);
      Serial.print(" v -- ");
      
  
      float iout = (float)(uint16_t(rxBuf[6] << 8) + uint16_t(rxBuf[7])) / 28;
      //Serial.print(rxBuf[7], HEX);
      //Serial.print(rxBuf[8], HEX);
      Serial.print(" ");
      Serial.print(iout);
      Serial.println(" A");
   }else{

      //print out the message
  

      if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "%.8lX:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "%.3lX,false,0,%1d", rxId, len);
  
      Serial.print(msgString);
  
      if ((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for (byte i = 0; i < len; i++) {
          sprintf(msgString, " %.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
      
      if(rxId == 0x00860F8007){
        long unsigned int bob = (uint32_t(rxBuf[4]) << 16) + (uint32_t(rxBuf[5]) << 8) + rxBuf[6];
        
        //sprintf(msgString, " --- %1d", bob);
        Serial.print(" --  ");
        Serial.print(rxBuf[3], HEX);
        Serial.print(":  ");
        Serial.print(bob); // / 0x40000 + 32);
  
        
      }
  
      if(rxBuf[3] == 0x23) Serial.print(" -- 23");
  
      Serial.println();
      
      if(rxId == 0x00860F8003 && rxBuf[3] == 0x58){
        //delay(200);
        if(rxBuf[4] == 0x46){
          sendCurrentPerc(0x0607FF83);
        }else{
          sendControl();
          gimme5();
        }
        
        
   
      }
  

  
      if(rxBuf[3] == 0x5D && initialising && got5B){
        initialising = 0;
        delay(500);
        sendControl();
      }
  
      if(rxBuf[3] == 0x5B) got5B=1;
      if(rxBuf[3] == 0x23 && !sentBothSyncs) {
        sentBothSyncs = 1;
        sendSync();
        sendSync2();
      }
    
    
  }
}

/*
 * Convert an hex string to binary. Spaces are allowed between bytes.
 * The output array is supposed to be large enough.
 * On return, *size is the size of the byte array.
 */
static void hex2bin(uint8_t *out, const char *in, size_t *size)
{
    size_t sz = 0;
    while (*in) {
        while (*in == ' ') in++;  // skip spaces
        if (!*in) break;
        uint8_t c = *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        *out++ = c;
        sz++;
    }
    *size = sz;
}

static void hex82bin(long unsigned int &out, const char *in)
{

    while (*in) {
        while (*in == ' ') in++;  // skip spaces
        if (!*in) break;
        long unsigned int c = *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        c <<= 4;
        if (!*in) break;
        c |= *in>='a' ? *in-'a'+10 : *in>='A' ? *in-'A'+10 : *in-'0';
        in++;
        out = c;
        
    }
    
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
