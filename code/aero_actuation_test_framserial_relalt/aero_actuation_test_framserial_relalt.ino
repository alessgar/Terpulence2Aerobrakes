#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>

#include "aerobuzzer.h"
#include "sdcard.h"
#include "fram.h"
#include "timing.h"
#include "actuation.h"
#include "aero_gps.h"

#define GPSSerial Serial1               // Serial Port used for hardware Serial Transmission

// Constants
#define LIFTOFF_GS 4.0f
#define LIFTOFF_HEIGHT 15.24f
#define ACTUATION_HEIGHT 304.8f

#define SEALEVELPRESSURE_HPA (1018.287) // The pressure level of the area, used to get altitude but wont affect relative alt.

// Sensor Objects
Adafruit_BMP3XX bmp;
Adafruit_ICM20649 icm;
// Adafruit_GPS GPS(&GPSSerial);

float timeNow = 0.0f;                   // Used to hold current time
float lastTimeNow = 0.0f;               // Used for delta time
float startingHeight = 0;               // used for relative altitude
bool isLaunched = false;                // Used for launch triggers;

float startingAccelX = 0;               // used for zeroed out acceleration
float startingAccelY = 0;               // used for zeroed out acceleration
float startingAccelZ = 0;               // used for zeroed out acceleration
float timeSinceLaunchAccelCondMet = -1.0f;     // used for launch condition

float lastBuzz = 0.0f;

// Holds initialization state of sensor
bool bmpReady = false;
bool icmReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Actuation Test Program!"));

  setupTeensySerial();

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
    icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);;

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
  if(!setupGPS()){
    if(!failCode){
      failCode = 3;
    }
  }

  // Setup FRAM Module
  if(!setupFram()){
    if(!failCode){
      failCode = 4;
    }
  }

  // Setup SD Card and log file
  if (!setupSDCard()) {
    if(!failCode){
      failCode = 5;
    }
  }

  // Get first reading
  Serial.println(F("Initalization complete! Beginning feedback loop..."));
  Serial.println(F("Serial console here on out will only be utilized when actuation conditions are met"));
  soundBuzz(failCode + 1);
  timeNow = millis() / (1000.0f);
  setStartTime(timeNow);
  lastBuzz = timeNow;
  
  Serial.print("Event Log:\n");
}

void loop() {
  // Start of row data
  lastTimeNow = timeNow;
  timeNow = millis() / (1000.0f);

  // Check for Launch
  if(icmReady && bmpReady){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){   
      //if (!isLaunched && (timeNow - getStartTime()) >= 10.0f) {
      //if(!isLaunched && bmp.temperature >= 28.0f){
      bool accelerationConditions = ((startingAccelZ - accel.acceleration.z) >= LIFTOFF_GS && timeSinceLaunchAccelCondMet >= 0.0f && (timeNow - timeSinceLaunchAccelCondMet) >= 0.25f);
      if(!isLaunched && accelerationConditions){
        Serial.println(F("Liftoff!"));
        setLaunchTime(timeNow);
        isLaunched = true;
      }else if(!isLaunched && (startingAccelZ - accel.acceleration.z) >= LIFTOFF_GS && timeSinceLaunchAccelCondMet <= 0.0f){
        timeSinceLaunchAccelCondMet = timeNow;
      }else if(!isLaunched && (startingAccelZ - accel.acceleration.z) < LIFTOFF_GS){
        timeSinceLaunchAccelCondMet = -1.0f;
      }
    }
  }

  // Pre-Launch Data collection - keep about 30 seconds of flight data stored before the launch
  if(!isLaunched && !isFramDumped() && getFramNextLoc() > 0){
    framDumpToSD();
  }
  
  if(!isFramDumped()){
    startRow(timeNow);
  }

  // Buzz every 10 seconds
  if(timeNow - lastBuzz >= 10.0f){
    lastBuzz = timeNow;
    soundBuzz(1);
  }

  if(!isFramDumped()){
    // Get BMP Data
    outputBMP();
  
    // Get IMU Data
    outputIMU();
  
    // Get GPS Data
    outputGPS();
  }

  // Actuation Test - open flaps when temp is greater than 50 C
  if (bmpReady && icmReady && isLaunched) {
    //if (!getIsActuating() && timeNow - getStartTime() >= 20.0f) {
    //if (!getIsActuating() && bmp.temperature >= 35.0f) {
    if (!getIsActuating() && (bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight) >= ACTUATION_HEIGHT && isLaunched && (timeNow - getLaunchTime()) >= 1.0f){
    //if (!getIsActuating() && (bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight) >= ACTUATION_HEIGHT){
      setDesiredActuation(1080.0f);
      setIsActuating(true);
      setLastActuated(timeNow);
      Serial.println(F("Actation conditions met! Acutating flaps and closing in 3 seconds..."));
    } else if(getIsActuating() && timeNow - getLastActuated() > 3.0f){  // 3 Second delay before closing
      if(!getHasActuated() && isOneTimeActuate() || !isOneTimeActuate()){
        setDesiredActuation(0.0f);
        Serial.println(F("De-Actuating..."));
        if(!isOneTimeActuate()){
          setIsActuating(false);
        }
        setHasActuated(true);
      }
    }
  }

  // Actuate flaps as needed
  rotateFlaps();
  if(!isFramDumped()){
    outputActuation();

    // End of row data
    endRow();
  }

  // Post Launch Condition - Dump FRAM to SD Card
  if(!isFramDumped() && isLaunched && (timeNow - getLaunchTime() >= 100.0f)){ // record 100 seconds at launch
    framDumpToSD();
  }
}

// Adds BMP data to the CSV row
void outputBMP() {
  if (bmpReady && isFramReady()) {
    /* Get temp, pressure, and altitude */
    if (bmp.performReading()) {
      insertBlankValues(1);
      framPrint(bmp.temperature);

      insertBlankValues(1);
      framPrint(bmp.pressure / 100.0);

      insertBlankValues(1);
      framPrint(bmp.readAltitude(SEALEVELPRESSURE_HPA));

      insertBlankValues(1);
      framPrint(bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight);
    } else {
      insertBlankValues(4);
    }
  } else {
    insertBlankValues(4);
  }
}

// Adds IMU data to the CSV row
void outputIMU() {
  if (icmReady && isFramReady()) {
    /* Get temp, pressure, and altitude */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
      insertBlankValues(1);
      framPrint(startingAccelX - accel.acceleration.x);

      insertBlankValues(1);
      framPrint(startingAccelY - accel.acceleration.y);

      insertBlankValues(1);
      framPrint(startingAccelZ - accel.acceleration.z);

      insertBlankValues(1);
      framPrint(gyro.gyro.x);

      insertBlankValues(1);
      framPrint(gyro.gyro.y);

      insertBlankValues(1);
      framPrint(gyro.gyro.z);
    } else {
      insertBlankValues(6);
    }
  } else {
    insertBlankValues(6);
  }
}
