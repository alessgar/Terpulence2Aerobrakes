#ifndef AERO_IMU
#define AERO_IMU

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include "fram.h"
#include <imuFilter.h>

constexpr float GAIN = 0.1;

bool setupIMU();                                // Initialize the sensor
void calibrateIMU();                            // Calibrate the sensor for relative motion

void outputIMU();                               // Output sensor data to the CSV
bool isIMUReady();                              // Returns whether the sensor is initialized

float getRelAccelX();                           // Returns the relative X acceleration
float getRelAccelY();                           // Returns the relative Y acceleration
float getRelAccelZ();                           // Returns the relative Z acceleration

float getPitch();                           // Returns the pitch in radians
float getYaw();                           // Returns the yaw in radians
float getRoll();                           // Returns the roll in radians

#endif
