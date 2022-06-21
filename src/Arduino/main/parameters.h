#ifndef PARAMETERS
#define PARAMETERS

//Desired flight profile
#define DESIRED_APOGEE 2900.0f //9500ft
#define ACTUATION_HEIGHT 2150.0f //7000ft
#define FLIGHT_TIME 30.0f //seconds

//Controller gains
//#define Kp 0.12e-3
//#define Ki 3.8e-6 //1e-6

#define Kp 0.2e-3
#define Ki 3.3e-4 //1e-6

//lift-off paramters
#define LIFTOFF_GS 30.0f // m/s^2 accelerometer data
#define LIFTOFF_HEIGHT 20.0f

//sensor parameters
#define IMU_CALLIBRATION_TIME 10.0f
#define IMU_CALLIBRATION_SAMPLES 50 //KEEP THIS AT 50!!!

#endif
