#ifndef PARAMETERS
#define PARAMETERS

//Desired flight profile
#define DESIRED_APOGEE 3048.0f //10000ft
#define ACTUATION_HEIGHT 2150.0f //7000ft
#define FLIGHT_TIME 35.0f //seconds //Time from launch to apogee to descent 
                                    //also used for timing the fRAM dump 

//Safety parameters
#define TILT_SET_TIME 2.0f //seconds
#define MAX_TILT 30.0f //in degrees  Right now this is overridden and the flaps will always actuate and caps the final tilt at 15d

//Controller gains
//#define Kp 0.12e-3
//#define Ki 3.8e-6 //1e-6

#define Kp 0.2e-3
#define Ki 3.3e-4 //1e-6

//lift-off paramters
#define LIFTOFF_GS 30.0f // m/s^2 accelerometer data
#define LIFTOFF_HEIGHT 10.0f //meters

//sensor parameters
#define IMU_CALLIBRATION_TIME 20.0f //seconds
#define IMU_CALLIBRATION_SAMPLES 50 //KEEP THIS AT 50!!!

#endif
