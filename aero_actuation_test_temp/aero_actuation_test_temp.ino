#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <Buzzer.h>
#include "SdFat.h"
#include "sdios.h"

// Constants
#define SEALEVELPRESSURE_HPA (1018.287) // The pressure level of the area, used to get altitude but wont affect relative alt.
#define sd_FAT_TYPE 0                   // Used to select the storage type we want to use, 0 just means FAT16/FAT32, 1 is FAT16/FAT32, 2 is exFat, and 3 is both
#define SPI_SPEED SD_SCK_MHZ(50)        // How fast should the SD Card writing be? Slow down to 10-20 for breadboards, otherwise 50 when soldered
#define CS_PIN 10                       // What pin is used for CS? Used for SD Card
#define GPSSerial Serial1               // Serial Port used for hardware Serial Transmission
#define BUZZER_PIN 13                   // What pin is the buzzer inserted into?

// Motor Constants
#define MOTOR_DIRECTION_PIN 2
#define MOTOR_STEP_PIN 3
#define MOTOR_ENABLE_PIN 4
#define MOTOR_STEPS_PER_REVOLUTION 250
#define MOTOR_SPEED 400                 // microseconds between each step; 200 microseconds is fastest with accurate movement, going below will make it too fast
#define MOTOR_MAX_DEGREES 1080.0        // Maximum actuation
#define MOTOR_MIN_DEGREES 0.0           // Minimum actuation
#define MOTOR_ANGLE_MOVEMENT 1800.0     // How many degrees to move at a time max

// Sensor Objects
Adafruit_BMP3XX bmp;
Adafruit_ICM20948 icm;
Adafruit_GPS GPS(&GPSSerial);
Buzzer buzzer(BUZZER_PIN);

float startingHeight = 0;               // used for relative altitude
float startTime = 0.0f;                 // Used for relative timestamps

float curActuation = 0.0f;              // Used to track active actuation state
float desiredActuation = 0.0f;          // Used to track desired actuation state
bool isActuating = false;
float lastActuated = 0.0f;

// Variables for logging
SdFat sd;
File logFile;
String logFileName;

// Holds initialization state of sensor
bool bmpReady = false;
bool sdReady = false;
bool icmReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Actuation Test Program!"));

  // Setup Actuator Flaps
  // Declare pins as Outputs
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); //disable motor by default

  int failCount = 0; // Fail counter for beep at end

  // Setup BMP Sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    failCount++;
  } else {
    Serial.println(F("BMP3 Sensor Found!"));
    bmpReady = true;

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

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
    failCount++;
  }

  // Setup IMU Sensor
  if (!icm.begin_I2C()) {
    Serial.println(F("Failed to find ICM20948 chip"));
    failCount++;
  }else{
    Serial.println(F("ICM20948 Found!"));
    icmReady = true;
  }

  // Setup GPS
  if(GPS.available()){
    GPS.begin(9600);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    Serial.println(F("GPS Found and Initialized!"));
  }else{
    Serial.println(F("GPS not found!"));
    failCount++;
  }

  // Setup SD Card and log file
  logFileName.reserve(24);
  Serial.println(sd.begin(10));
  Serial.println(sd.begin(CS_PIN, SPI_SPEED));
  if (sd.begin(CS_PIN, SPI_SPEED)) {
    // Find file name
    int fileNo = 1;
    bool exists = true;
    while (exists) {
      logFileName = "datalog_" + String(fileNo++) + ".csv";
      exists = sd.exists(logFileName);
    }

    // Setup file with CSV header
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.println(F("Time,BMP Temp,BMP Pressure,BMP Alt,BMP RelAlt,IMU Acceleration X,IMU Acceleration Y,IMU Acceleration Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,GPS Latitude,GPS Longitude,GPS Velocity,GPS Altitude,Desired Actuation,Cur Actuation"));
      logFile.close(); // close the file
      Serial.println("Log file created: " + logFileName);
    }else{
      Serial.println(F("SD Card reader found, but file was unable to be created"));
      failCount++;
    }

    sdReady = true;
  }else{
    Serial.println(F("SD Card Reader NOT found! Data will not be logged!"));
    failCount++;
  }

  // Get first reading
  Serial.println(F("Initalization complete! Beginning feedback loop..."));
  Serial.println(F("Serial console here on out will only be utilized when actuation conditions are met"));
  soundBuzzer(failCount);
  float timeNow = millis() / (1000.0f);
  startTime = timeNow;
  startRow(startTime);
  outputBMP();
  outputIMU();
  outputGPS();
  endRow();
}

void loop() {
  // Start of row data
  float timeNow = millis() / (1000.0f);
  startRow(timeNow);

  // Get BMP Data
  outputBMP();

  // Get IMU Data
  outputIMU();

  // Get GPS Data
  outputGPS();

  // Actuation Test - open flaps when temp is greater than 50 C
  if (bmpReady) {
    if (bmp.temperature >= 25.0f && !isActuating) {
      desiredActuation = 1080.0f;
      isActuating = true;
      lastActuated = timeNow;
      Serial.println(F("Actation conditions met! Acutating flaps and closing in 3 seconds..."));
    } else if(isActuating && timeNow - lastActuated > 3.0f){  // 3 Second delay before closing
      desiredActuation = 0.0f;
    }
  }

  // Actuate flaps as needed
  rotateFlaps();
  outputActuation();

  // End of row data
  endRow();
}

// inserts timestamp to start data row. Argument is timestamp
void startRow(float curTime) {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.print(curTime - startTime);
      logFile.close(); // close the file
    }
  }
}

// ends the row and adds a newline
void endRow() {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.println(); // Move to next data row
      logFile.close();
    }
  }
}

// Adds BMP data to the CSV row
void outputBMP() {
  if (bmpReady && sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      /* Get temp, pressure, and altitude */
      if (bmp.performReading()) {
        insertBlankValues(1);
        logFile.print(bmp.temperature);

        insertBlankValues(1);
        logFile.print(bmp.pressure / 100.0);

        insertBlankValues(1);
        logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));

        insertBlankValues(1);
        logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) - startingHeight);
      } else {
        insertBlankValues(4);
      }

      logFile.close();
    }
  } else {
    insertBlankValues(4);
  }
}

// Adds IMU data to the CSV row
void outputIMU() {
  if (icmReady && sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      /* Get temp, pressure, and altitude */
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t mag;
      sensors_event_t temp;
      if(icm.getEvent(&accel, &gyro, &temp, &mag)){
        insertBlankValues(1);
        logFile.print(accel.acceleration.x);

        insertBlankValues(1);
        logFile.print(accel.acceleration.y);

        insertBlankValues(1);
        logFile.print(accel.acceleration.z);

        insertBlankValues(1);
        logFile.print(gyro.gyro.x);

        insertBlankValues(1);
        logFile.print(gyro.gyro.y);

        insertBlankValues(1);
        logFile.print(gyro.gyro.z);
      } else {
        insertBlankValues(6);
      }

      logFile.close();
    }
  } else {
    insertBlankValues(4);
  }
}

// Adds GPS data to CSV row
float cacheLatitude = 0.0f;
float cacheLongitude = 0.0f;
char cacheLatitudeDir = 'N';
char cacheLongitudeDir = 'E';
float cacheSpeed = 0.0f;
float cacheAltitude = 0.0f;
void outputGPS(){
  if(GPS.available()){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      GPS.read();
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another

        // Update GPS Data
        if (GPS.fix) {
          cacheLatitude = GPS.latitudeDegrees;
          cacheLatitudeDir = GPS.lat;
          cacheLongitude = GPS.longitudeDegrees;
          cacheLongitudeDir = GPS.lon;
          cacheSpeed = GPS.speed * 1.15077945;
          cacheAltitude = GPS.altitude;
        }
      }

      // Print data to CSV
      insertBlankValues(1);
      logFile.print(cacheLatitude);
      logFile.print(cacheLatitudeDir);

      insertBlankValues(1);
      logFile.print(cacheLongitude);
      logFile.print(cacheLongitudeDir);

      insertBlankValues(1);
      logFile.print(cacheSpeed);

      insertBlankValues(1);
      logFile.print(cacheAltitude);

      logFile.close();
    }
  } else {
    insertBlankValues(4);
  }
}

// Adds actuation data to CSV row
void outputActuation() {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      insertBlankValues(1);
      logFile.print(desiredActuation);
      insertBlankValues(1);
      logFile.print(curActuation);


      logFile.close();
    } else {
      insertBlankValues(2);
    }
  } else {
    insertBlankValues(2);
  }
}

// Adds the specified number of blank values to the CSV row. Argument is # of blank values to insert
void insertBlankValues(int numValues) {
  if (sdReady) {
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      for (int i = 0; i < numValues; i++) {
        logFile.print(F(",")); // Have blank data when sensor not found
      }

      logFile.close(); // close the file
    }
  }
}

// Wrapper for motor functions, to prevent misuse and allow for gradual actuation and adjustment between sensor readings
void rotateFlaps() {
  // Figure out how much we WANT to actuate and in what direction
  float diff = curActuation - desiredActuation;
  bool isOpening = false;
  if (diff < 0.0f) {
    isOpening = true;
    diff *= -1;
  }

  // Cap how much we want to actuate to prevent delaying other code for too long
  float angle = diff;
  if (angle > MOTOR_ANGLE_MOVEMENT) {
    angle = MOTOR_ANGLE_MOVEMENT;
  }

  // Do nothing if actuation would violate bounds
  if (isOpening && curActuation + angle > MOTOR_MAX_DEGREES) {
    return;
  } else if (!isOpening && curActuation - angle < MOTOR_MIN_DEGREES) {
    return;
  }

  // If no issues found, and angle is substantial enough, actuate in the needed direction
  if (angle > 0.1f) {
    if (isOpening) {
      rotateCounterClockwise(angle);
    } else {
      rotateClockwise(angle);
    }
  }
}

// close the flaps
void rotateClockwise(float angle)
{
  float steps = (angle / 360) * MOTOR_STEPS_PER_REVOLUTION;
  // Set motor direction clockwise
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
  // Enable the motor
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  for (int x = 0; x < steps; x++)
  {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED); //don't go below 200!!!
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  // Disable the motor
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  curActuation -= angle;
}

// Open the flaps
void rotateCounterClockwise(float angle)
{
  float steps = (angle / 360) * MOTOR_STEPS_PER_REVOLUTION;
  // Set motor direction counter-clockwise
  digitalWrite(MOTOR_DIRECTION_PIN, LOW);
  // Enable the motor
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  for (int x = 0; x < steps; x++)
  {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED); //don't go below 200!!!
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  // Disable the motor
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  curActuation += angle;
}

// Sound the Buzzer
void soundBuzzer(int totalBeeps){
  buzzer.begin(0);

  for(int i = 0; i < totalBeeps; i++){
    buzzer.sound(NOTE_G3, 500);
    delay(100);
  }
  
  buzzer.end(0);
}
