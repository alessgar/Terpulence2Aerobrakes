#include "aero_bmp.h"

// Sensors and other related variables
Adafruit_BMP3XX bmp;
bool bmpReady = false;                  // Tracks whether the sensor is ready
float startingHeight = 0;               // used for relative altitude

// Initialize the sensor
bool setupBMP(){
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    return false;
  } else {
    Serial.println(F("BMP3 Sensor Found!"));
    bmpReady = true;

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    calibrateBMP();
  }

  return true;
}


// Output data from the sensor to the CSV
void outputBMP(){
  if (bmpReady && isFramReady()) {
    /* Get temp, pressure, and altitude */
    if (bmp.performReading()) {
      //insertBlankValues(1);
      //framPrint(bmp.temperature);

      //insertBlankValues(1);
      //framPrint(bmp.pressure / 100.0);

      insertBlankValues(1);
      framPrint(bmp.readAltitude(SEALEVELPRESSURE_HPA));

      insertBlankValues(1);
      framPrint(bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight);

      // Estimated Height
      insertBlankValues(1);
      framPrint(0.0);
    } else {
      insertBlankValues(3);
    }
  } else {
    insertBlankValues(3);
  }
}

// Calibrate our relative altitude
void calibrateBMP(){
  // Calibrate relative height
  if (bmpReady) {
    Serial.println(F("Calibrating height..."));
    for (int i = 0; i < 5; i++) {
      if (bmp.performReading()) {
        startingHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }
      
      delay(1000);
    }
    Serial.println(F("Height calibration Complete!"));
  }else{
    Serial.println(F("Height calibration skipped due to BMP missing!"));
  }
}

// Return whether the BMP is initialized
bool isBMPReady(){
    return bmpReady;
}

// Returns the direct altitude reported by the sensor
float getExactAltitude(){
    return bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

// Returns the relative altitude from when the sensor was initialized
float getRelAltitude(){
    return getExactAltitude() - startingHeight;
}
