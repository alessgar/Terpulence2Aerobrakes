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
  if(isSDReady() && framReady){
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
      
      //soundBuzz(1);
  }else{
    if(getLaunchTime() > getStartTime()){
      soundBuzz(2);
    }
  }

  framNextLoc = 0;
  isFRAMDumped = true;
}

// inserts timestamp to start data row. Argument is timestamp
void startRow(float curTime) {
  if (framReady) {
    framPrint(curTime - getStartTime());

    insertBlankValues(1);
    if(getLaunchTime() < 0.1f){
      framPrint("0.00");
    }else{
      framPrint(curTime - getLaunchTime());
    }

    insertBlankValues(1);
    if(getLastActuated() < 0.1f){
      framPrint("0.00");
    }else{
      framPrint(curTime - getLastActuated());
    }
  }
}

// ends the row and adds a newline
void endRow() {
  if (framReady) {
    framPrintln(); // Move to next data row
  }
}

// Returns whether the FRAM is initialized
bool isFramReady(){
    return framReady;
}

// Returns whether the FRAM has been dumped
bool isFramDumped(){
    return isFRAMDumped;
}

// Returns the next free memory location in FRAM
int getFramNextLoc(){
    return framNextLoc;
}

// Returns true if next location is at 90% of total capacity
float getCapacity(){
    return framNextLoc / 512000.0f;
}

// Initializes the FRAM
bool setupFram(){
    pinMode(FRAM_CS_PIN, OUTPUT);
    digitalWrite(FRAM_CS_PIN, HIGH);

    if(fram.begin(3)){  // 256KB uses fram.begin() ; 512KB uses fram.begin(3) 
        //Serial.println(F("FRAM Ready"));
        framReady = true;

        framPrintln(F("Program Uptime,Time Since Launch,Time Since Last Actuation,Barometer,Altitude,Filtered Height,Filtered Velocity,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z,Roll,Pitch,Heading,Tilt,Desired Actuation"));
    }else{
        //Serial.println(F("FRAM not found"));
       
        return false;
    }

    return true;
}

// Adds the specified number of blank values to the CSV row. Argument is # of blank values to insert
void insertBlankValues(int numValues) {
  if (isFramReady()) {
    for (int i = 0; i < numValues; i++) {
      framPrint(F(",")); // Have blank data when sensor not found
    }
  }
}

void resetDumpStatus(){
  isFRAMDumped = false;
}
