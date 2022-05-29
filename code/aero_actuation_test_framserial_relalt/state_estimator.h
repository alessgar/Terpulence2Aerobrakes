#ifndef AERO_STATE_ESTIMATOR
#define AERO_STATE_ESTIMATOR

void updateState();

float getTilt();
float getHeight();
float getVelocity();

void outputStateEstimates();
#endif
