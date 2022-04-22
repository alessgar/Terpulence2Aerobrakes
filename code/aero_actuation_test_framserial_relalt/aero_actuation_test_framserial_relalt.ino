#include "aerobuzzer.h"
#include "sdcard.h"
#include "fram.h"
#include "timing.h"
#include "actuation.h"
#include "aero_gps.h"
#include "aero_bmp.h"
#include "aero_imu.h"

// Constants
#define LIFTOFF_GS 4.0f
#define LIFTOFF_HEIGHT 15.24f
#define ACTUATION_HEIGHT 304.8f

bool oneTimeActuate = true;             // Should we only actuate once?

float timeNow = 0.0f;                   // Used to hold current time
float lastTimeNow = 0.0f;               // Used for delta time
bool isLaunched = false;                // Used for launch triggers;

float timeSinceLaunchAccelCondMet = -1.0f;     // used for launch condition

float lastBuzz = 0.0f;

// Holds initialization state of sensor
bool icmReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Actuation Test Program!"));

  setupTeensySerial();

  int failCode = 0; // Fail counter for beep at end

  // Setup BMP Sensor
  if(!setupBMP()){
    if(!failCode){
      failCode = 1;
    }
  }

  // Setup IMU Sensor
  if(!setupIMU()){
    if(!failCode){
      failCode = 2;
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

  // Buzz every 10 seconds
  if(timeNow - lastBuzz >= 10.0f){
    lastBuzz = timeNow;
    soundBuzz(1);
  }

  // Check for Launch
  if(isIMUReady() && isBMPReady()){  
    //if (!isLaunched && (timeNow - getStartTime()) >= 10.0f) {
    //if(!isLaunched && bmp.temperature >= 28.0f){
    bool accelerationConditions = (getRelAccelZ() >= LIFTOFF_GS && timeSinceLaunchAccelCondMet >= 0.0f && (timeNow - timeSinceLaunchAccelCondMet) >= 0.25f);
    if(!isLaunched && accelerationConditions){
      Serial.println(F("Liftoff!"));
      setLaunchTime(timeNow);
      isLaunched = true;
    }else if(!isLaunched && getRelAccelZ() >= LIFTOFF_GS && timeSinceLaunchAccelCondMet <= 0.0f){
      timeSinceLaunchAccelCondMet = timeNow;
    }else if(!isLaunched && getRelAccelZ() < LIFTOFF_GS){
      timeSinceLaunchAccelCondMet = -1.0f;
    }
  }

  // Pre-Launch Data collection - keep about 30 seconds of flight data stored before the launch
  if(!isLaunched && !isFramDumped() && getFramNextLoc() > 0){
    framDumpToSD();
  }

  // Start a new row in our CSV
  if(!isFramDumped()){
    startRow(timeNow);
  }

  // Data Output to our CSV
  if(!isFramDumped()){
    // Get BMP Data
    outputBMP();
  
    // Get IMU Data
    outputIMU();
  
    // Get GPS Data
    outputGPS();
  }

  // Actuation Test - open flaps when temp is greater than 50 C
  if (isBMPReady() && isIMUReady() && isLaunched) {
    //if (!getIsActuating() && timeNow - getStartTime() >= 20.0f) {
    //if (!getIsActuating() && bmp.temperature >= 35.0f) {
    if (!getIsActuating() && getRelAltitude() >= ACTUATION_HEIGHT && isLaunched && (timeNow - getLaunchTime()) >= 1.0f){
    //if (!getIsActuating() && getRelAltitude() >= ACTUATION_HEIGHT){
      setDesiredActuation(1080.0f);
      setIsActuating(true);
      setLastActuated(timeNow);
      Serial.println(F("Actation conditions met! Acutating flaps and closing in 3 seconds..."));
    } else if(getIsActuating() && timeNow - getLastActuated() > 3.0f){  // 3 Second delay before closing
      if(!getHasActuated() && oneTimeActuate || !oneTimeActuate){
        setDesiredActuation(0.0f);
        Serial.println(F("De-Actuating..."));
        if(!oneTimeActuate){
          setIsActuating(false);
        }
        setHasActuated(true);
      }
    }
  }

  // Actuate flaps as needed (send data to teensy)
  rotateFlaps();
  
  if(!isFramDumped()){
    // Output our current actuation
    outputActuation();

    // End of row data
    endRow();
  }

  // Post Launch Condition - Dump FRAM to SD Card
  if(!isFramDumped() && isLaunched && (timeNow - getLaunchTime() >= 100.0f)){ // record 100 seconds at launch
    framDumpToSD();
  }
}
