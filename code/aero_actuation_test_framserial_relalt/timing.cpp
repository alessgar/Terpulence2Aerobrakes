#include "timing.h"

float startTime = 0.0f;                 // Used for relative timestamps
float launchTime = 0.0f;                // Used for relative launch timestamp
float lastActuated = 0.0f;

void setStartTime(float input){
    startTime = input;
}

void setLaunchTime(float input){
    launchTime = input;
}

void setLastActuated(float input){
    lastActuated = input;
}

float getStartTime(){
    return startTime;
}

float getLaunchTime(){
    return launchTime;
}

float getLastActuated(){
    return lastActuated;
}