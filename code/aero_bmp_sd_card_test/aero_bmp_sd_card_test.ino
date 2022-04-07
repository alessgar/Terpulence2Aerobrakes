#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SdFat.h"
#include "sdios.h"

// Constants
#define SEALEVELPRESSURE_HPA (1018.287) // The pressure level of the area, used to get altitude but wont affect relative alt.
#define sd_FAT_TYPE 0                   // Used to select the storage type we want to use, 0 just means FAT16/FAT32, 1 is FAT16/FAT32, 2 is exFat, and 3 is both
#define SPI_SPEED SD_SCK_MHZ(50)        // How fast should the SD Card writing be? Slow down to 10-20 for breadboards, otherwise 50 when soldered
#define CS_PIN 10                       // What pin is used for CS? Used for SD Card

// Sensor Objects
Adafruit_BMP3XX bmp;

float startingHeight = 0;               // used for relative altitude
float startTime = 0.0f;                 // Used for relative timestamps

bool flapsOpen = false;                 // Used to track whether flaps are open

// Variables for logging
SdFat sd;
File logFile;
String logFileName;

// Holds initialization state of sensor
bool bmpReady = false;
bool sdReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Setup BMP Sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
  }else{
    Serial.println(F("BMP3 Sensor Found!"));
    bmpReady = true;
    
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // Calibrate relative height
  if(bmpReady){
    for(int i = 0; i < 5; i++){      
      if(bmp.performReading()){
        startingHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }

      delay(1000);
    }
  }

  // Setup SD Card and log file
  logFileName.reserve(24);
  if (sd.begin(CS_PIN, SPI_SPEED)) {
    // Find file name
    int fileNo = 1;
    bool exists = true;
    while(exists){
      logFileName = "datalog_" + String(fileNo++) + ".csv";
      exists = sd.exists(logFileName);
    }
    Serial.println(logFileName);

    // Setup file with CSV header
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {    
      logFile.println(F("Time,BMP Temp,BMP Pressure,BMP Alt,BMP RelAlt"));
      logFile.close(); // close the file
      Serial.println(F("File Created"));
    }

    sdReady = true;
  }

  // Get first reading
  float timeNow = millis()/(1000.0f);
  startTime = timeNow;
  startRow(startTime);
  outputBMP();
  endRow();
}

void loop() {
  // Start of row data
  float timeNow = millis()/(1000.0f);
  startRow(timeNow);

  // Get BMP Data
  outputBMP();

  // End of row data
  endRow();
}

// inserts timestamp to start data row. Argument is timestamp
void startRow(float curTime){
  if(sdReady){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {    
      logFile.print(curTime - startTime);
      logFile.close(); // close the file
    }
  }
}

// ends the row and adds a newline
void endRow(){
  if(sdReady){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) { 
      logFile.println(); // Move to next data row
      logFile.close();
    }
  }
}

// Adds BMP data to the CSV row
void outputBMP(){
  if(bmpReady && sdReady){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) { 
      /* Get temp, pressure, and altitude */
      if (bmp.performReading()) {
        insertBlankValues(1);
        logFile.print(bmp.temperature);
      
        insertBlankValues(1);
        logFile.print(bmp.pressure / 100.0);
      
        insertBlankValues(1);
        logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    
        insertBlankValues(1);
        logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight);
      }else{
        insertBlankValues(4);
      }
      
      logFile.close();
    }
  }else{
    insertBlankValues(4);
  }
}

// Adds the specified number of blank values to the CSV row. Argument is # of blank values to insert
void insertBlankValues(int numValues){
  if(sdReady){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {    
      for(int i = 0; i < numValues; i++){
        logFile.print(F(",")); // Have blank data when sensor not found
      }
      
      logFile.close(); // close the file
    }
  }
}
