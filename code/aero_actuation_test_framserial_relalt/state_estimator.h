#ifndef AERO_STATE_ESTIMATOR
#define AERO_STATE_ESTIMATOR

void updateState(float);

float getTilt();
float getHeight();
float getVelocity();
float simulateAccel(float);
float simulateHeight(float);

void outputStateEstimates();
#endif
