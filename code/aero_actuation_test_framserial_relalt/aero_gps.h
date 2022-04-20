#ifndef AERO_GPS
#define AERO_GPS
#include <Adafruit_GPS.h>
#define GPSSerial Serial1

bool setupGPS();

void outputGPS();

#endif
