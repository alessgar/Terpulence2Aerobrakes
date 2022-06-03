#ifndef AERO_CONTROLLER
#define AERO_CONTROLLER

#include<math.h>
#include "aero_bmp.h"
#include "state_estimator.h"

#define Kp 7e-4
#define Ki 1e-5
#define LIFTOFF_GS 30.0f

#define DESIRED_APOGEE 2900.0f //9500ft
#define ACTUATION_HEIGHT 1500.0f //5000ft

float calcDeflection(float, float); //time and previousTime

#endif
