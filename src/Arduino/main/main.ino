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
#include "sim_variables.h"

#include "parameters.h"

float timeNow = 0.0f;                   // Used to hold current time
float lastTimeNow = 0.0f;               // Used for delta time
float lastBuzz = 0.0f;                  // Used during pre-launch

bool isLaunched = false;                // Used for launch triggers
bool launchConditions = false;          // condition checks for launch

float finalTilt = 0.0f;       //used to fix tilt measured after burnout
bool tiltSet = false;         //keep track of whether finalTilt is set or not

float brakeDeflection = 0.0f;

// Holds initialization state of sensor
bool icmReady = false;

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  //Serial.println(F("Starting Actuation Test Program!"));

  setupTeensySerial();

  int failCode = 0; // Fail counter for beep at end

  // Setup BMP Sensor
  if(!setupBMP() && !failCode){
      failCode = 1;
    }

  // Setup IMU Sensor
  if(!setupIMU() && !failCode){
      failCode = 2;
    }

  // Setup GPS
  if(!setupGPS() && !failCode){
      failCode = 3;
    }

  // Setup FRAM Module
  if(!setupFram() && !failCode){
      failCode = 4;
    }

  // Setup SD Card and log file
  if (!setupSDCard() && !failCode) {
      failCode = 5;
    }
  
  timeNow = millis() / (1000.0f);
  setStartTime(timeNow);

  soundBuzz(failCode + 1);
  lastBuzz = timeNow;
}

void loop() {
  //Getting timestamps
  lastTimeNow = timeNow;
  timeNow = millis() / (1000.0f);

  float simTime = timeNow-startTime;//-600;
  float deltasimTime=timeNow-lastTimeNow;
  float simHeight = sim_Height(simTime);
  float simAccel = sim_Accel();
  //Serial.println(simHeight*3.28);
  
  // Buzz every 10 seconds before launching
  if(!isLaunched && (timeNow - lastBuzz >= 10.0f)){
    lastBuzz = timeNow;
    soundBuzz(1);
  }

  // Writing to fRAM and then right away to SD Card
  if(!isLaunched && !isFramDumped() && getFramNextLoc() > 0){     //writing to SD card before launch
    framDumpToSD(); resetDumpStatus();
  }else if(isLaunched && isFramDumped() && getFramNextLoc() > 0){ //writing to SD card during descent
    framDumpToSD();
  }

  // Start a new row in our datalog CSV
  if(!isFramDumped() || isLaunched){
    startRow(timeNow);
  }

  // Check for Launch
  if(!isLaunched){  
    //real flight
    //launchConditions = (abs(getRelAccelY()) > LIFTOFF_GS && (getRelAltitude() > LIFTOFF_HEIGHT));
    
    //simulated flight
    launchConditions = (simHeight > LIFTOFF_HEIGHT) && (abs(simAccel) > LIFTOFF_GS);
    
    if(launchConditions){
      setLaunchTime(timeNow);
      isLaunched = true;
    }
  }

   Serial.print(simTime); Serial.print("= simTime, ");
   Serial.print(simTime-getLaunchTime()); Serial.print("= timesincelaunch, ");
   Serial.print(simHeight); Serial.print(" = simHeight");
   //Serial.print(simAccel); Serial.print(" ");
   //Serial.print(accelerationConditions); Serial.print(" ");
   //Serial.println(getDesiredActuation());
    
  //keep updating the state every iteration 
  //updateState(timeNow, lastTimeNow); //EKF
  computeOrientation();

  //Compute the tilt of rocket after burnout
  //if(!tiltSet && (timeNow - getLaunchTime() > TILT_SET_TIME)) {
  if(!tiltSet && (simTime - getLaunchTime() > TILT_SET_TIME)) {
    finalTilt = returnTilt();
    tiltSet = true;
  }
  
  Serial.print(finalTilt); Serial.print("= finalTilt; ");
  Serial.print(returnTilt()); Serial.println("= currentTilt; ");


  
  // computing deflection angle and sending to Teensy
  if (isLaunched && !isFramDumped()) {
    //if (getRelAltitude() > ACTUATION_HEIGHT && getRelAltitude() < DESIRED_APOGEE && finalTilt < MAX_TILT){
    if (simHeight > ACTUATION_HEIGHT && simHeight < DESIRED_APOGEE && finalTilt < MAX_TILT) {
      
      //brakeDeflection = calcDeflection(timeNow, lastTimeNow);
      brakeDeflection = calcDeflection(simTime, simTime-deltasimTime);
      
      setDesiredActuation(brakeDeflection);
      setIsActuating(true);
      setLastActuated(timeNow);    
    }
  }

  //Deflect to the max if its past apogee  
  //if(getRelAltitude()>DESIRED_APOGEE && !isFramDumped()){
  if(simHeight>DESIRED_APOGEE && !isFramDumped()){
     setDesiredActuation(83.0f);
  }

  // Actuate flaps as needed (send data to teensy)
  rotateFlaps();

  // Writing to fRAM
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
  if(!isFramDumped() && isLaunched && (getCapacity() > 0.9f || timeNow - getLaunchTime() > FLIGHT_TIME)) {
    
    setDesiredActuation(0.0f);
    rotateFlaps();
    
    framDumpToSD();
  }
}
