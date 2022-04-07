#include "SdFat.h"
#include "sdios.h"
#include "Adafruit_FRAM_SPI.h"

// Constants
#define sd_FAT_TYPE 0                   // Used to select the storage type we want to use, 0 just means FAT16/FAT32, 1 is FAT16/FAT32, 2 is exFat, and 3 is both
#define SPI_SPEED SD_SCK_MHZ(50)        // How fast should the SD Card writing be? Slow down to 10-20 for breadboards, otherwise 50 when soldered
#define SD_CS_PIN 10                       // What pin is used for CS? Used for SD Card
#define FRAM_CS_PIN A4                       // What pin is used for FRAM? Used for FRAM

// Sensor Objects
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS_PIN);

uint32_t framNextLoc = 0;                  // Next open FRAM location for writing
bool isFRAMDumped = false;               // Checked for when FRAM dump condition met

// Variables for logging
SdFat sd;
File logFile;
String logFileName;

// Holds initialization state of sensor
bool sdReady = false;
bool framReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting FRAM Dump Program!"));

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(FRAM_CS_PIN, OUTPUT);
  digitalWrite(FRAM_CS_PIN, HIGH);

  // Setup FRAM Module
  // 256KB uses fram.begin() ; 512KB uses fram.begin(3) 
  if(fram.begin()){
    Serial.println(F("FRAM Ready"));
    framReady = true;
  }else{
    Serial.println(F("FRAM not found"));
  }

  digitalWrite(FRAM_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, LOW);

  // Setup SD Card and log file
  logFileName.reserve(24);
  if (sd.begin(SD_CS_PIN, SPI_SPEED)) {
    // Find file name
    int fileNo = 1;
    bool exists = true;
    while (exists) {
      logFileName = "framdump_" + String(fileNo++) + ".csv";
      exists = sd.exists(logFileName);
    }

    // Setup file with CSV header
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.close(); // close the file
      Serial.println("Test file created: " + logFileName);
      sdReady = true;
    }else{
      Serial.println(F("SD Card reader found, but file was unable to be created"));
    }
  }else{
    Serial.println(F("SD Card Reader NOT found!"));
  }

  digitalWrite(FRAM_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);

  if(framReady && sdReady){
    framDumpToSD();
    Serial.println(F("Dump successful!"));
  }else{
    Serial.println(F("Test failed due to missing FRAM"));
  }
}

void loop() {
  
}

// Dump FRAM to SD Card
void framDumpToSD(){
  if(sdReady && framReady){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      Serial.println("Writing " + String(framNextLoc) + " bytes...");
      
      for(int i = 0; i < framNextLoc; i++){
        logFile.print(fram.read8(i));
        if (i % 10000 == 0){
          Serial.println(String(i + 1) + "/" + String(framNextLoc));
        }
      }

      logFile.close(); // close the file
    }else{
      Serial.println(F("Failed to open file for writing!"));
    }
  }
}
