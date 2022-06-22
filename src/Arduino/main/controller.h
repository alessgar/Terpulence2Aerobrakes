#ifndef AERO_CONTROLLER
#define AERO_CONTROLLER

#include<math.h>
#include "aero_bmp.h"
#include "state_estimator.h"
#include "aero_imu.h"
#include "sim_variables.h"
#include "parameters.h"

float calcDeflection(float, float, float); //time and previousTime

#endif
