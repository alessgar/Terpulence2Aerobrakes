#ifndef AERO_BMP
#define AERO_BMP

#include "Adafruit_BMP3XX.h"
#include <Adafruit_Sensor.h>
#include "fram.h"

#define SEALEVELPRESSURE_HPA (1018.287) // The pressure level of the area, used to get altitude but wont affect relative alt.

bool setupBMP();              // Initialize the sensor
void calibrateBMP();          // Calibrate our relative altitude

void outputBMP();             // Output data from the sensor to the CSV
bool isBMPReady();            // Return whether the BMP is initialized

float getExactAltitude();     // Returns the direct altitude reported by the sensor
float getRelAltitude();       // Returns the relative altitude from when the sensor was initialized

#endif
