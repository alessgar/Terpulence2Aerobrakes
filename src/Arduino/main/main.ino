#include "aerobuzzer.h"
#include "sdcard.h"
#include "fram.h"
#include "timing.h"
#include "actuation.h"
#include "aero_gps.h"
#include "aero_bmp.h"
#include "aero_imu.h"
#include "state_estimator.h"
#include "controller.h"

#define LIFTOFF_GS 30.0f
#define LIFTOFF_HEIGHT 20.0f

bool oneTimeActuate = true;             // Should we only actuate once?

float timeNow = 0.0f;                   // Used to hold current time
float lastTimeNow = 0.0f;               // Used for delta time
bool isLaunched = false;                // Used for launch triggers;

float timeSinceLaunchAccelCondMet = -1.0f;     // used for launch condition

float lastBuzz = 0.0f;
float brakeDeflection = 0.0f;

// Holds initialization state of sensor
bool icmReady = false;

void setup() {
  //Serial.begin(115200);
  //while (!Serial);
  //Serial.println(F("Starting Actuation Test Program!"));

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
  //Serial.println(F("Initalization complete! Beginning feedback loop..."));
  //Serial.println(F("Serial console here on out will only be utilized when actuation conditions are met"));
  soundBuzz(failCode + 1);
  timeNow = millis() / (1000.0f);
  setStartTime(timeNow);
  lastBuzz = timeNow;
  
  //Serial.print("Event Log:\n");
}

void loop() {
  //Getting timestamps
  lastTimeNow = timeNow;
  timeNow = millis() / (1000.0f);

  // Buzz every 10 seconds before launching
  if(!isLaunched && (timeNow - lastBuzz >= 10.0f)){
    lastBuzz = timeNow;
    soundBuzz(1);
  }

  // Pre-Launch Data collection -  record to SD During Launch
  if(!isLaunched && !isFramDumped() && getFramNextLoc() > 0){
    framDumpToSD(); resetDumpStatus();
  }else if(isLaunched && isFramDumped() && getFramNextLoc() > 0){
    framDumpToSD();
  }

  // Start a new row in our datalog CSV
  if(!isFramDumped() || isLaunched){
    startRow(timeNow);
  }

  // Check for Launch
  if(isIMUReady() && isBMPReady()){  
    //if (!isLaunched && (timeNow - getStartTime()) > 15.0f) { //time-based launch detection for simulation
    //bool accelerationConditions = (getRelAccelX() > LIFTOFF_GS && timeSinceLaunchAccelCondMet >= 0.0f && (timeNow - timeSinceLaunchAccelCondMet) >= 0.25f) || timeNow - getStartTime() > 10.0f;
    bool accelerationConditions = (abs(getRelAccelX()) > LIFTOFF_GS && (getRelAltitude() > LIFTOFF_HEIGHT));
    if(!isLaunched && accelerationConditions){
      setLaunchTime(timeNow);
      isLaunched = true;
    }else if(!isLaunched && abs(getRelAccelX()) > LIFTOFF_GS && timeSinceLaunchAccelCondMet <= 0.0f){
      timeSinceLaunchAccelCondMet = timeNow;
    }else if(!isLaunched && abs(getRelAccelX()) < LIFTOFF_GS){
      timeSinceLaunchAccelCondMet = -1.0f;
    }
  }

  //keep updating the state every iteration 
  updateState(timeNow, lastTimeNow);
  
  // computing deflection angle
  if (isBMPReady() && isIMUReady() && isLaunched) {
    //if (!getIsActuating() && timeNow - getStartTime() >= 20.0f) {
    if (!getIsActuating() && getRelAltitude() > ACTUATION_HEIGHT && getRelAltitude() < DESIRED_APOGEE){
 
      brakeDeflection = calcDeflection(timeNow, lastTimeNow);
      setDesiredActuation(brakeDeflection);
      
      setIsActuating(true);
      setLastActuated(timeNow);    
    }
    else{
      setDesiredActuation(0.0f);
      setIsActuating(false);   
    }
    }
    
    if(getRelAltitude()>DESIRED_APOGEE || timeNow - getLastActuated() > 20.0f){  // 20 second total delay before closing, if it hasn't closed till now
    //if(simulateHeight(timeNow)>DESIRED_APOGEE){
        setDesiredActuation(0.0f);   
    }

  // Actuate flaps as needed (send data to teensy)
  rotateFlaps();

  // Data Output to our CSV
  if(!isFramDumped() || isLaunched){
    //Write sensor data and EKF estimates
    outputBMP(); outputStateEstimates(); outputIMU(); //outputGPS();

    // Output our current actuation
    outputActuation();

    // End of row data
    endRow();
  }

  // Post Launch Condition - Dump FRAM to SD Card
  //if(!isFramDumped() && isLaunched && (timeNow - getLaunchTime() >= 100.0f)){ // record 100 seconds at launch
  // Dump if Fram isn't dumped and either the capacity is > 90% or descending
  if(!isFramDumped() && isLaunched && (getCapacity() > 0.9f || timeNow - getLaunchTime() > 30.0f)) {
    framDumpToSD();
    //Serial.println("Dumped!");
  }
}
