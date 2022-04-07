#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <Buzzer.h>
#include "SdFat.h"
#include "sdios.h"
#include "wiring_private.h"

// Constants
#define SEALEVELPRESSURE_HPA (1018.287) // The pressure level of the area, used to get altitude but wont affect relative alt.
#define sd_FAT_TYPE 0                   // Used to select the storage type we want to use, 0 just means FAT16/FAT32, 1 is FAT16/FAT32, 2 is exFat, and 3 is both
#define SPI_SPEED SD_SCK_MHZ(50)        // How fast should the SD Card writing be? Slow down to 10-20 for breadboards, otherwise 50 when soldered
#define CS_PIN 10                       // What pin is used for CS? Used for SD Card
#define GPSSerial Serial1               // Serial Port used for hardware Serial Transmission
#define BUZZER_PIN A0                   // What pin is the buzzer inserted into?

// Motor Constants
#define MOTOR_MAX_DEGREES 1080.0        // Maximum actuation
#define MOTOR_MIN_DEGREES 0.0           // Minimum actuation
#define MOTOR_ANGLE_MOVEMENT 1800.0     // How many degrees to move at a time max

#define LIFTOFF_GS 2.0f
#define LIFTOFF_HEIGHT 15.24f
#define ACTUATION_HEIGHT 304.8f

// Sensor Objects
Adafruit_BMP3XX bmp;
Adafruit_ICM20649 icm;
Adafruit_GPS GPS(&GPSSerial);
Buzzer buzzer(BUZZER_PIN);

float timeNow = 0.0f;                   // Used to hold current time
float lastTimeNow = 0.0f;               // Used for delta time
float startingHeight = 0;               // used for relative altitude
float startTime = 0.0f;                 // Used for relative timestamps
float launchTime = 0.0f;                // Used for relative launch timestamp
bool isLaunched = false;                // Used for launch triggers;

float startingAccelX = 0;               // used for zeroed out acceleration
float startingAccelY = 0;               // used for zeroed out acceleration
float startingAccelZ = 0;               // used for zeroed out acceleration
float timeSinceLaunchAccelCondMet = -1.0f;     // used for launch condition

bool oneTimeActuate = true;             // Should we only actuate once?
bool hasActuated = false;               // Used to enforce single time actuation

float lastBuzz = 0.0f;

float desiredActuation = 0.0f;          // Used to track desired actuation state
bool isActuating = false;
float lastActuated = 0.0f;

// Setup for Teensy Serial
#define PIN_SERIAL2_RX       (2ul)               // Pin description number for PIO_SERCOM on D12 -> pin 1 on teensy
#define PIN_SERIAL2_TX       (3ul)               // Pin description number for PIO_SERCOM on D10 -> pin 0 on teensy
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_2)    // SERCOM pad 3
Uart Serial2 (&sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

// Variables for logging
SdFat sd;
File logFile;
String logFileName;

// Holds initialization state of sensor
bool bmpReady = false;
bool sdReady = false;
bool icmReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Actuation Test Program!"));

  // Setup Teensy Comms
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM);
  Serial2.begin(115200);
  while(!Serial2);

  int failCode = 0; // Fail counter for beep at end

  // Setup BMP Sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    if(!failCode){
      failCode = 1;
    }
  } else {
    Serial.println(F("BMP3 Sensor Found!"));
    bmpReady = true;

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // Calibrate relative height
  if (bmpReady) {
    Serial.println(F("Calibrating height..."));
    for (int i = 0; i < 5; i++) {
      if (bmp.performReading()) {
        startingHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }
      
      delay(1000);
    }
    Serial.println(F("Height calibration Complete!"));
  }else{
    Serial.println(F("Height calibration skipped due to BMP missing!"));
  }

  // Setup IMU Sensor
  if (!icm.begin_I2C()) {
    Serial.println(F("Failed to find ICM20948 chip"));
    if(!failCode){
      failCode = 2;
    }
  }else{
    Serial.println(F("ICM20948 Found!"));
    icmReady = true;

    // Set IMU settings
    icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
    icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){
      startingAccelX = accel.acceleration.x;
      startingAccelY = accel.acceleration.y;
      startingAccelZ = accel.acceleration.z;
    }else{
      icmReady = false;
      Serial.println(F("ICM20948 Failed to Calibrate!"));
      if(!failCode){
        failCode = 2;
      }
    }
  }

  // Setup GPS
  GPS.begin(9600);
  delay(3000);
  if(GPS.available()){
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    Serial.println(F("GPS Found and Initialized!"));
  }else{
    Serial.println(F("GPS not found!"));
    if(!failCode){
      failCode = 3;
    }
  }

  // Setup SD Card and log file
  logFileName.reserve(24);
  if (sd.begin(CS_PIN, SPI_SPEED)) {
    // Find file name
    int fileNo = 1;
    bool exists = true;
    while (exists) {
      logFileName = "datalog_" + String(fileNo++) + ".csv";
      exists = sd.exists(logFileName);
    }

    // Setup file with CSV header
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.println(F("Program Uptime,Time Since Launch,Time Since Last Actuation,BMP Temp,BMP Pressure,BMP Alt,BMP RelAlt,IMU Acceleration X,IMU Acceleration Y,IMU Acceleration Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,GPS Latitude,GPS Longitude,GPS Velocity,GPS Altitude,Desired Actuation"));
      logFile.close(); // close the file
      Serial.println("Log file created: " + logFileName);
      sdReady = true;
    }else{
      Serial.println(F("SD Card reader found, but file was unable to be created"));
      if(!failCode){
        failCode = 4;
      }
    }
  }else{
    Serial.println(F("SD Card Reader NOT found! Data will not be logged!"));
    if(!failCode){
      failCode = 4;
    }
  }

  // Get first reading
  Serial.println(F("Initalization complete! Beginning feedback loop..."));
  Serial.println(F("Serial console here on out will only be utilized when actuation conditions are met"));
  soundBuzzer(failCode + 1);
  timeNow = millis() / (1000.0f);
  startTime = timeNow;
  lastBuzz = timeNow;
  startRow(startTime);
  outputBMP();
  outputIMU();
  outputGPS();
  outputActuation();
  endRow();
  
  Serial.print("Event Log:\n");
}

void loop() {
  // Start of row data
  lastTimeNow = timeNow;
  timeNow = millis() / (1000.0f);
  startRow(timeNow);

  // Buzz every 10 seconds
  if(timeNow - lastBuzz >= 10.0f){
    lastBuzz = timeNow;
    soundBuzzer(1);
  }

  // Get BMP Data
  outputBMP();

  // Get IMU Data
  outputIMU();

  // Get GPS Data
  outputGPS();

  // Check for Launch
  /*if(icmReady && bmpReady){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){   
      //if (!isLaunched && (timeNow - startTime) >= 10.0f) {
      //if(!isLaunched && bmp.temperature >= 28.0f){
      Serial.println("!!!");
      Serial.println((startingAccelZ - accel.acceleration.z));
      Serial.println((timeNow - timeSinceLaunchAccelCondMet));
      Serial.println((bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight));
      bool accelerationConditions = ((startingAccelZ - accel.acceleration.z) >= LIFTOFF_GS && timeSinceLaunchAccelCondMet >= 0.0f && (timeNow - timeSinceLaunchAccelCondMet) >= 0.5f);
      bool heightCondition = ((bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight) >= LIFTOFF_HEIGHT);
      if(!isLaunched && accelerationConditions && heightCondition){
        Serial.println(F("Liftoff!"));
        launchTime = timeNow;
        isLaunched = true;
      }else if(!isLaunched && (startingAccelZ - accel.acceleration.z) >= LIFTOFF_GS && timeSinceLaunchAccelCondMet <= 0.0f){
        timeSinceLaunchAccelCondMet = timeNow;
      }else if(!isLaunched && (startingAccelZ - accel.acceleration.z) < LIFTOFF_GS){
        timeSinceLaunchAccelCondMet = -1.0f;
      }
    }
  }*/

  // Actuation Test - open flaps when temp is greater than 50 C
  if (bmpReady) {
    //if (!isActuating && timeNow - startTime >= 20.0f) {
    //if (!isActuating && bmp.temperature >= 35.0f) {
    //if (!isActuating && (bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight) >= ACTUATION_HEIGHT && isLaunched && (timeNow - launchTime) >= 1.0f){
    if (!isActuating && (bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight) >= ACTUATION_HEIGHT){
      desiredActuation = 1080.0f;
      isActuating = true;
      lastActuated = timeNow;
      Serial.println(F("Actation conditions met! Acutating flaps and closing in 3 seconds..."));
    } else if(isActuating && timeNow - lastActuated > 3.0f){  // 3 Second delay before closing
      if(!hasActuated && oneTimeActuate || !oneTimeActuate){
        desiredActuation = 0.0f;
        Serial.println(F("De-Actuating..."));
        if(!oneTimeActuate){
          isActuating = false;
        }
        hasActuated = true;
      }
    }
  }

  // Actuate flaps as needed
  rotateFlaps();
  outputActuation();

  // End of row data
  endRow();
}

// inserts timestamp to start data row. Argument is timestamp
void startRow(float curTime) {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.print(curTime - startTime);

      insertBlankValuesNoClose(1);
      if(launchTime < 0.1f){
        logFile.print("0.00");
      }else{
        logFile.print(curTime - launchTime);
      }

      insertBlankValuesNoClose(1);
      if(lastActuated < 0.1f){
        logFile.print("0.00");
      }else{
        logFile.print(curTime - lastActuated);
      }
      
      logFile.close(); // close the file
    }
  }
}

// ends the row and adds a newline
void endRow() {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.println(); // Move to next data row
      logFile.close();
    }
  }
}

// Adds BMP data to the CSV row
void outputBMP() {
  if (bmpReady && sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      /* Get temp, pressure, and altitude */
      if (bmp.performReading()) {
        insertBlankValuesNoClose(1);
        logFile.print(bmp.temperature);

        insertBlankValuesNoClose(1);
        logFile.print(bmp.pressure / 100.0);

        insertBlankValuesNoClose(1);
        logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));

        insertBlankValuesNoClose(1);
        logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight);
      } else {
        insertBlankValuesNoClose(4);
      }

      logFile.close();
    }
  } else {
    insertBlankValues(4);
  }
}

// Adds IMU data to the CSV row
void outputIMU() {
  if (icmReady && sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      /* Get temp, pressure, and altitude */
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t mag;
      sensors_event_t temp;
      if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
        insertBlankValuesNoClose(1);
        logFile.print(startingAccelX - accel.acceleration.x);

        insertBlankValuesNoClose(1);
        logFile.print(startingAccelY - accel.acceleration.y);

        insertBlankValuesNoClose(1);
        logFile.print(startingAccelZ - accel.acceleration.z);

        insertBlankValuesNoClose(1);
        logFile.print(gyro.gyro.x);

        insertBlankValuesNoClose(1);
        logFile.print(gyro.gyro.y);

        insertBlankValuesNoClose(1);
        logFile.print(gyro.gyro.z);
      } else {
        insertBlankValuesNoClose(6);
      }

      logFile.close();
    }
  } else {
    insertBlankValues(6);
  }
}

// Adds GPS data to CSV row
float cacheLatitude = 0.0f;
float cacheLongitude = 0.0f;
char cacheLatitudeDir = 'N';
char cacheLongitudeDir = 'E';
float cacheSpeed = 0.0f;
float cacheAltitude = 0.0f;
void outputGPS(){
  if(GPS.available()){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      GPS.read();
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another

        // Update GPS Data
        if (GPS.fix) {
          cacheLatitude = GPS.latitudeDegrees;
          cacheLatitudeDir = GPS.lat;
          if(cacheLatitudeDir == 'S'){
            cacheLatitude *= -1.0f;
          }
          
          cacheLongitude = GPS.longitudeDegrees;
          cacheLongitudeDir = GPS.lon;
          if(cacheLongitudeDir == 'W'){
            cacheLongitude *= -1.0f;
          }

          cacheSpeed = GPS.speed * 1.15077945;
          cacheAltitude = GPS.altitude;
        }
      }

      // Print data to CSV
      insertBlankValuesNoClose(1);
      logFile.print(cacheLatitude);
      logFile.print(cacheLatitudeDir);

      insertBlankValuesNoClose(1);
      logFile.print(cacheLongitude);
      logFile.print(cacheLongitudeDir);

      insertBlankValuesNoClose(1);
      logFile.print(cacheSpeed);

      insertBlankValuesNoClose(1);
      logFile.print(cacheAltitude);

      logFile.close();
    }
  } else {
    insertBlankValues(4);
  }
}

// Adds actuation data to CSV row
void outputActuation() {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      insertBlankValuesNoClose(1);
      logFile.print(desiredActuation);

      logFile.close();
    }
  } else {
    insertBlankValues(2);
  }
}

// Adds the specified number of blank values to the CSV row. Argument is # of blank values to insert
void insertBlankValues(int numValues) {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      for (int i = 0; i < numValues; i++) {
        logFile.print(F(",")); // Have blank data when sensor not found
      }

      logFile.close(); // close the file
    }
  }
}

// Same as insertBlankValues, but assumes file opening/closing is already handled
void insertBlankValuesNoClose(int numValues) {
  if (logFile) {
    for (int i = 0; i < numValues; i++) {
      logFile.print(F(",")); // Have blank data when sensor not found
    }
  }
}

// Output our desired actuation to the teensy
void rotateFlaps() {
  Serial2.println(desiredActuation);
}

// Sound the Buzzer
void soundBuzzer(int totalBeeps){
  buzzer.begin(BUZZER_PIN);

  for(int i = 0; i < totalBeeps; i++){
    buzzer.sound(NOTE_G3, 500);
    delay(100);
  }
  
  buzzer.end(0);
}

void SERCOM2_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}
