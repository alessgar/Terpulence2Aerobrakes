#include "aero_imu.h"

// Create sensor instance.
DPEng_ICM20948 dpEng = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };
float orientation_offsets[3] = { 0.0F, 0.0F, 0.0F };

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -1.76F, 22.54F, 4.43F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.954,  -0.019,  0.003 },
                                    {  -0.019,  1.059, -0.009 },
                                    {  0.003,  -0.009,  0.990 } };

float mag_field_strength        = 29.85F;


// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony_DPEng filter;
//Madgwick_DPEng filter;

bool imuReady = false;                  // Whether the sensor is initialized

float roll = 0.0f;
float pitch = 0.0f;
float heading = 0.0f;
float tilt = 0.0f;

// Initialize the sensor
bool setupIMU(){
  // Initialize the sensors.
  if(!dpEng.begin(ICM20948_ACCELRANGE_16G, GYRO_RANGE_250DPS, ICM20948_ACCELLOWPASS_50_4_HZ)) {
    /* There was a problem detecting the ICM20948 ... check your connections */
    //Serial.println("Ooops, no ICM20948/AK09916 detected ... Check your wiring!");
    return false;
  }
  else {
    imuReady = true;
  }

  calibrateIMU();
  filter.begin();

  //run till the filter converges
  while(millis()/1000.0f < IMU_CALLIBRATION_TIME) { //wait for 10s to make the filter converge
    computeOrientation();
  }
  
  computeOrientationOffsets();

  soundBuzz(3);
  delay(200);
  soundBuzz(2);
  delay(200);
  soundBuzz(1);
  return true;
}


// Calibrate the sensor for relative motion
void calibrateIMU(){

    float startingGyroX = 0.0f; 
    float startingGyroY = 0.0f;
    float startingGyroZ = 0.0f;

    //float startingMagX = 0.0f; 
    //float startingMagY = 0.0f;
    //float startingMagZ = 0.0f;
    
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t magneto;
    
    for(int i = 0; i < IMU_CALLIBRATION_SAMPLES; i++){
        if(dpEng.getEvent(&accel, &gyro, &magneto)){
      
          //startingMagX += magneto.magnetic.x;
          //startingMagY += magneto.magnetic.y;
          //startingMagZ += magneto.magnetic.z;
    
          startingGyroX +=  gyro.gyro.x;
          startingGyroY +=  gyro.gyro.y;
          startingGyroZ +=  gyro.gyro.z;
        }
    }   

    //mag_offsets[0] = startingMagX/50.0f;
    //mag_offsets[1] = startingMagY/50.0f;
    //mag_offsets[2] = startingMagZ/50.0f;
    
    gyro_zero_offsets[0] = startingGyroX/50.0f;
    gyro_zero_offsets[1] = startingGyroY/50.0f;
    gyro_zero_offsets[2] = startingGyroZ/50.0f;
}


// Output sensor data to the CSV
void outputIMU(){
  
  if (imuReady && isFramReady()) {
      
    sensors_event_t accel_event;
    sensors_event_t gyro_event;
    sensors_event_t mag_event;

    // Get new data samples
    if(dpEng.getEvent(&accel_event, &gyro_event, &mag_event)){

    // Apply mag offset compensation (base values in uTesla)
    float x = mag_event.magnetic.x - mag_offsets[0];
    float y = mag_event.magnetic.y - mag_offsets[1];
    float z = mag_event.magnetic.z - mag_offsets[2];

    // Apply mag soft iron error compensation
    float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

    // Apply gyro zero-rate error compensation
    float gx = gyro_event.gyro.x - gyro_zero_offsets[0];
    float gy = gyro_event.gyro.y - gyro_zero_offsets[1];
    float gz = gyro_event.gyro.z - gyro_zero_offsets[2];

                      
      insertBlankValues(1); framPrint(accel_event.acceleration.x);
      insertBlankValues(1); framPrint(accel_event.acceleration.y);
      insertBlankValues(1); framPrint(accel_event.acceleration.z);

      insertBlankValues(1); framPrint(gx);
      insertBlankValues(1); framPrint(gy);
      insertBlankValues(1); framPrint(gz);

      // Update the filter
      filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
                
      roll = filter.getPitch() - orientation_offsets[0];
      pitch = filter.getRoll() - orientation_offsets[1];
      heading = filter.getYaw() - orientation_offsets[2];
      tilt = atan(sqrt(pow(tan(roll*PI/180.0F),2)+pow(tan(pitch*PI/180.0F),2)))*180.0F/PI;
  
      // Pitch, Yaw, Roll, Tilt
      insertBlankValues(1); framPrint(roll);
      insertBlankValues(1); framPrint(pitch);
      insertBlankValues(1); framPrint(heading);
      insertBlankValues(1); framPrint(tilt);

  }
  }else {
    insertBlankValues(11);
  }
}



void computeOrientation(){
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;

  // Get new data samples
  dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
}


void computeOrientationOffsets(){
    // Setup sensor fusion with gravity vector

    float pitchBias = 0.0f;
    float rollBias = 0.0f;
    float yawBias = 0.0f;
    
    for(int i = 0; i < IMU_CALLIBRATION_SAMPLES; i++){
      computeOrientation();

      rollBias += filter.getPitch();
      pitchBias += filter.getRoll();
      yawBias += filter.getYaw();
    }

    orientation_offsets[0] = rollBias/50.0F;
    orientation_offsets[1] = pitchBias/50.0F;
    orientation_offsets[2] = yawBias/50.0F;
}

// Returns whether the sensor is initialized
bool isIMUReady(){
    return imuReady;
}

// Returns the relative X acceleration
float getRelAccelX(){
    if (imuReady) {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t mag_event;
        if(dpEng.getEvent(&accel_event, &gyro_event, &mag_event)){        
            //return startingAccelX - accel.acceleration.x;
            return accel_event.acceleration.x;
        }
    }
}

// Returns the relative Y acceleration
float getRelAccelY(){
    if (imuReady) {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t mag_event;
        if(dpEng.getEvent(&accel_event, &gyro_event, &mag_event)){        
            //return startingAccelX - accel.acceleration.x;
            return accel_event.acceleration.y;
        }
    }
}

// Returns the relative Z acceleration
float getRelAccelZ(){
    if (imuReady) {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t mag_event;
        if(dpEng.getEvent(&accel_event, &gyro_event, &mag_event)){        
            //return startingAccelX - accel.acceleration.x;
            return accel_event.acceleration.z;
        }
    }
}

float returnRoll(){ //in degrees!
  return filter.getPitch() - orientation_offsets[0]; 
}

float returnPitch(){ //in degrees!
  return filter.getRoll() - orientation_offsets[1];
}

float returnHeading(){ //in degrees!
  return filter.getYaw() - orientation_offsets[2];
}

float returnTilt(){ //in degrees!
      float roll = filter.getPitch() - orientation_offsets[0];
      float pitch = filter.getRoll() - orientation_offsets[1];
      
      float tilt = atan(sqrt(pow(tan(roll*PI/180.0F),2)+pow(tan(pitch*PI/180.0F),2)))*180.0F/PI;
      return tilt;
}
