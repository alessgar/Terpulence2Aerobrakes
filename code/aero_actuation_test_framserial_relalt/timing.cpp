#include "timing.h"

float startTime = 0.0f;                 // Used for relative timestamps
float launchTime = 0.0f;                // Used for relative launch timestamp
float lastActuated = 0.0f;              // Used to keep track of when we last actuated

// Set the startup timestamp
void setStartTime(float input){
    startTime = input;
}

// Set the launch timestamp
void setLaunchTime(float input){
    launchTime = input;
}

// Set the last actuation timestamp
void setLastActuated(float input){
    lastActuated = input;
}

// Return our startup timestamp
float getStartTime(){
    return startTime;
}

// Return our launch timestamp
float getLaunchTime(){
    return launchTime;
}

// Return our last actuation timestamp
float getLastActuated(){
    return lastActuated;
}
