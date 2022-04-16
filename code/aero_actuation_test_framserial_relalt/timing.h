#ifndef AERO_TIMING
#define AERO_TIMING

extern float startTime;                 // Used for relative timestamps
extern float launchTime;                // Used for relative launch timestamp
extern float lastActuated;

void setStartTime(float input);
void setLaunchTime(float input);
void setLastActuated(float input);

float getStartTime();
float getLaunchTime();
float getLastActuated();

#endif