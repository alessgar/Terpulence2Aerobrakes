#include "controller.h"
//#include <Arduino.h>

float heightError = 0.0f;
float integralError = 0.0f;
float controlInput = 0.0f;
float delta = 0.0f;
float actuationAngle = 0.0f;
float deltaTime = 0.0f;

float currentHeight = 0.0f;
float currentVelocity = 0.0f;
float currentTilt = 0.0f;

float calcDeflection(float timeNow, float lastTimeNow){

    currentHeight = sim_Height(timeNow); //simulated
    //currentHeight = getRelAltitude(); //from barometer
    //get the state feedback
    //currentHeight = getHeight(); //from EKF
    currentVelocity = getVelocity(); //from EKF
    currentTilt = returnTilt()*PI/180.0F;
    
    deltaTime = timeNow - lastTimeNow;
    heightError = (DESIRED_APOGEE - currentHeight)/cos(currentTilt);
    integralError = integralError + heightError*deltaTime; //(timeNow-lastTimeNow)
    controlInput = Kp*heightError + Ki*integralError;

    
    //Serial.println(controlInput);

    //controller clipping
    if (controlInput > 0.98f) {controlInput = 0.98f;} //max of 80 degrees
    if (controlInput < 0.0f) {controlInput = 0.0f;}
    
    delta = asin(controlInput) * 180.0f/PI;

      if(currentHeight<ACTUATION_HEIGHT){
        actuationAngle = 0.0f;
      }
      else{
      actuationAngle = delta;
      }
    
	return actuationAngle;
}
