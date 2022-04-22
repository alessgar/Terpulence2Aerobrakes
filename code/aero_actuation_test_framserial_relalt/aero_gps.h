#ifndef AERO_GPS
#define AERO_GPS

#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "fram.h"

#define GPSSerial Serial1

bool setupGPS();                  // Initialize the sensor

void outputGPS();                 // Adds GPS data to CSV row

bool isGPSReady();                // Returns whether the GPS is initialized

#endif
