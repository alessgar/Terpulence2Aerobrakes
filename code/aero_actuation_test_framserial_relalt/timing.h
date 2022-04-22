#ifndef AERO_TIMING
#define AERO_TIMING

extern float startTime;                 // Used for relative timestamps
extern float launchTime;                // Used for relative launch timestamp
extern float lastActuated;              // Used to keep track of when we last actuated

void setStartTime(float input);         // Set the startup timestamp
void setLaunchTime(float input);        // Set the launch timestamp
void setLastActuated(float input);      // Set the last actuation timestamp

float getStartTime();                   // Return our startup timestamp
float getLaunchTime();                  // Return our launch timestamp
float getLastActuated();                // Return our last actuation timestamp

#endif
