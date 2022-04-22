#ifndef AERO_IMU
#define AERO_IMU

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include "fram.h"

bool setupIMU();                                // Initialize the sensor
void calibrateIMU();                            // Calibrate the sensor for relative motion

void outputIMU();                               // Output sensor data to the CSV
bool isIMUReady();                              // Returns whether the sensor is initialized

float getRelAccelX();                           // Returns the relative X acceleration
float getRelAccelY();                           // Returns the relative Y acceleration
float getRelAccelZ();                           // Returns the relative Z acceleration

#endif
