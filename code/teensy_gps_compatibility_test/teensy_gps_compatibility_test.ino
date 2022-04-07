#include "wiring_private.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (3ul)               // Pin description number for PIO_SERCOM on D12 -> pin 1 on teensy
#define PIN_SERIAL2_TX       (2ul)               // Pin description number for PIO_SERCOM on D10 -> pin 0 on teensy
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)    // SERCOM pad 3

Uart Serial2 (&sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

#define GPSSerial Serial1               // Serial Port used for hardware Serial Transmission
Adafruit_GPS GPS(&GPSSerial);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Actuation Test Program!"));

  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM);
  Serial2.begin(115200);

  while(!Serial2);

  // Setup GPS
  GPS.begin(9600);
  delay(3000);
  if(GPS.available()){
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    Serial.println(F("GPS Found and Initialized!"));
  }else{
    Serial.println(F("GPS not found!"));
  }
}

void loop() {
  Serial2.println(random(100000) / 10.0f);

  if(GPS.available()){
    GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
  
      // Update GPS Data
      if (GPS.fix) {
        if(GPS.lat == 'S'){
          Serial.println(GPS.latitudeDegrees * -1.0f);
        }else{
          Serial.println(GPS.latitudeDegrees);
        }

        if(GPS.lon == 'W'){
          Serial.println(GPS.longitudeDegrees * -1.0f);
        }else{
          Serial.println(GPS.longitudeDegrees);
        }
  
        Serial.println(GPS.speed * 1.15077945);
        Serial.println(GPS.altitude);
      }
    }
  }
}

void SERCOM2_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}
