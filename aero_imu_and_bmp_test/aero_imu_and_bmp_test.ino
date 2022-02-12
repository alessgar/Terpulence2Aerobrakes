#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_BMP3XX.h"
#include <SPI.h>

// Constants
#define SEALEVELPRESSURE_HPA (1018.287) // Best to set this to the pressure of where the rocket will be launching

// Sensor Objects
Adafruit_ICM20948 icm;
Adafruit_BMP3XX bmp;

// Readiness variables to handle wobbly sensor connections pre-solder
bool bmpReady = false;
bool icmReady = false;

float startingHeight = 0; // used for relative altitude

void setup(void) {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Combined sensor test!"));

  // Initialize IMU
  imuSetup();
  
  // Initialize BMP Sensor
  bmpSetup();
  
  // Set starting height - we will use this to detect when we launch
  relAltSetup();
}

void loop() {  
  // Get and Print IMU Data
  imuOutput();

  // Get and Print BMP Data
  bmpOutput();
}

void imuSetup(){
  if (!icm.begin_I2C()) {
    Serial.println(F("Failed to find ICM20948 chip"));
  }else{
    Serial.println(F("ICM20948 Found!"));
    icmReady = true;
  }
}

void imuOutput(){
  if(icmReady){  
    Serial.println(F(" IMU Data: "));
    Serial.println();
    
    /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
  
    Serial.println(F(" deg C"));
    Serial.print(temp.temperature);
    Serial.println(F(" deg C"));
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print(F("\t\tAccel X: "));
    Serial.print(accel.acceleration.x);
    Serial.print(F(" \tY: "));
    Serial.print(accel.acceleration.y);
    Serial.print(F(" \tZ: "));
    Serial.print(accel.acceleration.z);
    Serial.println(F(" m/s^2 "));
  
    Serial.print(F("\t\tMag X: "));
    Serial.print(mag.magnetic.x);
    Serial.print(F(" \tY: "));
    Serial.print(mag.magnetic.y);
    Serial.print(F(" \tZ: "));
    Serial.print(mag.magnetic.z);
    Serial.println(F(" uT"));
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print(F("\t\tGyro X: "));
    Serial.print(gyro.gyro.x);
    Serial.print(F(" \tY: "));
    Serial.print(gyro.gyro.y);
    Serial.print(F(" \tZ: "));
    Serial.print(gyro.gyro.z);
    Serial.println(F(" radians/s "));
  }
}

void bmpSetup(){
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
  }else{
    Serial.println(F("BMP3 Sensor Found!"));
    bmpReady = true;
    
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
}

void bmpOutput(){
  if(bmpReady){
    Serial.println(F(" BMP388 Data: "));
    Serial.println();
    /* Get temp, pressure, and altitude */
    if (bmp.performReading()) {
      Serial.print(F("Temperature = "));
      Serial.print(bmp.temperature);
      Serial.println(F(" *C"));
    
      Serial.print(F("Pressure = "));
      Serial.print(bmp.pressure / 100.0);
      Serial.println(F(" hPa"));
    
      Serial.print(F("Est. Approx. Altitude = "));
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(F(" m"));
  
      Serial.print(F("Rel. Altitude = "));
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight);
      Serial.println(F(" m"));
    }else{
      insertBlankValues(4);
    }
  }
}

void relAltSetup(){
  if(bmpReady){
    for(int i = 0; i < 5; i++){      
      if(bmp.performReading()){
        startingHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }

      delay(1000);
    }
  }
}
