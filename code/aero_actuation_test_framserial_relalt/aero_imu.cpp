#include "aero_imu.h"

Adafruit_ICM20649 icm;
imuFilter <&GAIN> fusion;
bool imuReady = false;                  // Whether the sensor is initialized
float startingAccelX = 0;               // used for zeroed out acceleration
float startingAccelY = 0;               // used for zeroed out acceleration
float startingAccelZ = 0;               // used for zeroed out acceleration
float startingGyroX = 0;               // used for zeroed out gyro
float startingGyroY = 0;               // used for zeroed out gyro
float startingGyroZ = 0;               // used for zeroed out gyro
float pitchBias = 0;               // used for zeroed out gyro
float yawBias = 0;               // used for zeroed out gyro
float rollBias = 0;               // used for zeroed out gyro

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
    for(int i = 0; i < 5; i++){
        if(icm.getEvent(&accel, &gyro, &temp, &mag)){
      
          startingAccelX += accel.acceleration.x;
          startingAccelY += accel.acceleration.y;
          startingAccelZ += accel.acceleration.z;
    
          startingGyroX +=  gyro.gyro.x;
          startingGyroY +=  gyro.gyro.y;
          startingGyroZ +=  gyro.gyro.z;
        }else{
          imuReady = false;
          Serial.println(F("ICM20948 Failed to Calibrate!"));
        }
    }   

    startingAccelX /= 5;
    startingAccelY /= 5;
    startingAccelZ /= 5;
    startingGyroX /= 5;
    startingGyroY /= 5;
    startingGyroZ /= 5;

    // Setup sensor fusion with gravity vector
    for(int i = 0; i < 5; i++){
      fusion.setup( startingAccelX, startingAccelY, startingAccelZ );
      if(icm.getEvent(&accel, &gyro, &temp, &mag)){
        fusion.update( gyro.gyro.x - startingGyroX, gyro.gyro.y - startingGyroY, gyro.gyro.z - startingGyroZ, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z );  
      }else{
        imuReady = false;
        Serial.println(F("ICM20948 Failed to Calibrate!"));
      }
  
      pitchBias += fusion.pitch();
      yawBias += fusion.yaw();
      rollBias += fusion.roll();
    }

    pitchBias /= 5;
    yawBias /= 5;
    rollBias /= 5;
}

// Output sensor data to the CSV
void outputIMU(){
  if (imuReady && isFramReady()) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    if(icm.getEvent(&accel, &gyro, &temp, &mag)){        
      // Update sensor fusion
      fusion.update( gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z );  
      
      insertBlankValues(1);
      framPrint(accel.acceleration.x);

      insertBlankValues(1);
      framPrint(accel.acceleration.y);

      insertBlankValues(1);
      framPrint(accel.acceleration.z);

      insertBlankValues(1);
      framPrint((gyro.gyro.x - startingGyroX) * (180/PI));

      insertBlankValues(1);
      framPrint((gyro.gyro.y - startingGyroY) * (180/PI));

      insertBlankValues(1);
      framPrint((gyro.gyro.z - startingGyroZ) * (180/PI));

      // Pitch, Yaw, Roll
      insertBlankValues(1);
      framPrint((fusion.pitch() - pitchBias) * (180/PI));
      insertBlankValues(1);
      framPrint((fusion.roll() - rollBias) * (180/PI));
      insertBlankValues(1);
      framPrint((fusion.yaw() - yawBias) * (180/PI));
      Serial.println((fusion.pitch() - pitchBias) * (180/PI));
      Serial.println((fusion.roll() - rollBias) * (180/PI));
      Serial.println((fusion.yaw() - yawBias) * (180/PI));
        Serial.println();
      Serial.println(atan(sqrt(pow(tan(fusion.yaw() - yawBias), 2)+pow(tan(fusion.pitch() - pitchBias), 2))) * (180/PI));
      Serial.println();

      // Tilt
      insertBlankValues(1);
      framPrint(atan(sqrt(pow(tan(fusion.yaw() - yawBias), 2)+pow(tan(fusion.pitch() - pitchBias), 2))) * (180/PI));
    } else {
      insertBlankValues(11);
    }
  } else {
    insertBlankValues(11);
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
            //return startingAccelX - accel.acceleration.x;
            return accel.acceleration.x;
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
            //return startingAccelY - accel.acceleration.y;
            return accel.acceleration.y;
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
            //return startingAccelZ - accel.acceleration.z;
            return accel.acceleration.z;
        } else {
            Serial.println(F("Failed to get IMU Reading!"));
        }
    } else {
        Serial.println(F("Failed to get IMU Reading!"));
    }
}

float getPitch(){
  return fusion.pitch();
}

float getYaw(){
  return fusion.yaw();
}

float getRoll(){
  return fusion.roll();
}
