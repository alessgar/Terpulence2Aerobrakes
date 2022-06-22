#include "aero_gps.h"

Adafruit_GPS GPS(&GPSSerial); 
bool gpsready = false;            // Whether the sensor is initialized

// Initialize the sensor
bool setupGPS(){
  GPS.begin(9600);
  delay(3000);
  if(GPS.available()){
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    gpsready = true;
    //Serial.println(F("GPS Found and Initialized!"));
  }else{
    return false;
    /*Serial.println(F("GPS not found!"));
    if(!failCode){
      failCode = 3;
    }*/
  }
  return true;
}

// Adds GPS data to CSV row
float cacheLatitude = 0.0f;
float cacheLongitude = 0.0f;
char cacheLatitudeDir = 'N';
char cacheLongitudeDir = 'E';
float cacheSpeed = 0.0f;
float cacheAltitude = 0.0f;
void outputGPS(){
  if(GPS.available() && isFramReady()){
    GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another

      // Update GPS Data
      if (GPS.fix) {
        cacheLatitude = GPS.latitudeDegrees;
        cacheLatitudeDir = GPS.lat;
        if(cacheLatitudeDir == 'S'){
          cacheLatitude *= -1.0f;
        }
        
        cacheLongitude = GPS.longitudeDegrees;
        cacheLongitudeDir = GPS.lon;
        if(cacheLongitudeDir == 'W'){
          cacheLongitude *= -1.0f;
        }

        cacheSpeed = GPS.speed * 1.15077945;
        cacheAltitude = GPS.altitude;
      }
    }

    // Print data to CSV
    insertBlankValues(1);
    framPrint(cacheLatitude);
    framPrint(cacheLatitudeDir);

    insertBlankValues(1);
    framPrint(cacheLongitude);
    framPrint(cacheLongitudeDir);

    insertBlankValues(1);
    framPrint(cacheSpeed);

    insertBlankValues(1);
    framPrint(cacheAltitude);
  } else {
    insertBlankValues(4);
  }
}

// Returns whether the GPS is initialized
bool isGPSReady(){
  return gpsready;
}
