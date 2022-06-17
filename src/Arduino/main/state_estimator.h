#ifndef AERO_STATE_ESTIMATOR
#define AERO_STATE_ESTIMATOR

#include <BasicLinearAlgebra.h>
#include "aero_bmp.h"
#include "aero_imu.h"
#include "fram.h"
#include "sim_variables.h"

void updateState(float,float);

float getTilt();
float getHeight();
float getVelocity();
float simulateAccel(float);
float simulateHeight(float);

void outputStateEstimates();

#endif
