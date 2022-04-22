#include "aero_imu.h"

Adafruit_ICM20649 icm;
bool imuReady = false;                  // Whether the sensor is initialized
float startingAccelX = 0;               // used for zeroed out acceleration
float startingAccelY = 0;               // used for zeroed out acceleration
float startingAccelZ = 0;               // used for zeroed out acceleration

// Initialize the sensor
bool setupIMU(){
  if (!icm.begin_I2C()) {
    Serial.println(F("Failed to find ICM20948 chip"));
    return false;
  }else{
    Serial.println(F("ICM20948 Found!"));
    imuReady = true;

    // Set IMU settings
    icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
    icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);;

    calibrateIMU();
  }

  return true;
}

// Calibrate the sensor for relative motion
void calibrateIMU(){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){
      startingAccelX = accel.acceleration.x;
      startingAccelY = accel.acceleration.y;
      startingAccelZ = accel.acceleration.z;
    }else{
      imuReady = false;
      Serial.println(F("ICM20948 Failed to Calibrate!"));
    }
}

// Output sensor data to the CSV
void outputIMU(){
  if (imuReady && isFramReady()) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
      insertBlankValues(1);
      framPrint(startingAccelX - accel.acceleration.x);

      insertBlankValues(1);
      framPrint(startingAccelY - accel.acceleration.y);

      insertBlankValues(1);
      framPrint(startingAccelZ - accel.acceleration.z);

      insertBlankValues(1);
      framPrint(gyro.gyro.x);

      insertBlankValues(1);
      framPrint(gyro.gyro.y);

      insertBlankValues(1);
      framPrint(gyro.gyro.z);
    } else {
      insertBlankValues(6);
    }
  } else {
    insertBlankValues(6);
  }
}

// Returns whether the sensor is initialized
bool isIMUReady(){
    return imuReady;
}

// Returns the relative X acceleration
float getRelAccelX(){
    if (imuReady) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t mag;
        sensors_event_t temp;
        if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
            return startingAccelX - accel.acceleration.x;
        } else {
            Serial.println(F("Failed to get IMU Reading!"));
        }
    } else {
        Serial.println(F("Failed to get IMU Reading!"));
    }
}

// Returns the relative Y acceleration
float getRelAccelY(){
    if (imuReady) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t mag;
        sensors_event_t temp;
        if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
            return startingAccelY - accel.acceleration.y;
        } else {
            Serial.println(F("Failed to get IMU Reading!"));
        }
    } else {
        Serial.println(F("Failed to get IMU Reading!"));
    }
}

// Returns the relative Z acceleration
float getRelAccelZ(){
    if (imuReady) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t mag;
        sensors_event_t temp;
        if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
            return startingAccelZ - accel.acceleration.z;
        } else {
            Serial.println(F("Failed to get IMU Reading!"));
        }
    } else {
        Serial.println(F("Failed to get IMU Reading!"));
    }
}
