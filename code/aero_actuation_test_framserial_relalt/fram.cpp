#include "fram.h"

Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS_PIN);
uint32_t framNextLoc = 0;
bool isFRAMDumped = false;
bool framReady = false;

// Write newline to FRAM
void framPrintln(){
  fram.writeEnable(true);
  fram.write8(framNextLoc++, '\n');
  fram.writeEnable(false);
}

// Dump FRAM to SD Card
void framDumpToSD(){
  if(sdReady && framReady){
      String curStr = "";
      for(int i = 0; i < framNextLoc; i++){
        //Serial.println(String(i) + "/" + String(framNextLoc));
        char nextByte = fram.read8(i);
        curStr = curStr + nextByte;
        if(nextByte == '\n'){
          logFile = sd.open(logFileName, FILE_WRITE);
          if (logFile) {
            logFile.print(curStr);
            logFile.close(); // close the file
          }
          curStr = "";
        }
      }
      
      soundBuzz(1);
  }else{
    soundBuzz(3);
  }
}