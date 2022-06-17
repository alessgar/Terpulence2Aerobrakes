#ifndef AERO_IMU
#define AERO_IMU

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Mahony_DPEng.h"
#include "Madgwick_DPEng.h"
#include "DPEng_ICM20948_AK09916.h"
#include "fram.h"
#include "aerobuzzer.h"

#include "parameters.h"

//constexpr float GAIN = 0.1;

bool setupIMU();                                // Initialize the sensor
void calibrateIMU();                            // Calibrate the sensor for relative motion

void computeOrientation();
void computeOrientationOffsets();

void outputIMU();                               // Output sensor data to the CSV
bool isIMUReady();                              // Returns whether the sensor is initialized

float getRelAccelX();                           // Returns the relative X acceleration
float getRelAccelY();                           // Returns the relative Y acceleration
float getRelAccelZ();                           // Returns the relative Z acceleration

float returnRoll();                                // Returns the roll in radians
float returnPitch();                               // Returns the pitch in radians
float returnHeading();                             // Returns the yaw in radians       
float returnTilt();

#endif
