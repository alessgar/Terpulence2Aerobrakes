#include "sdcard.h"

SdFat sd;
File logFile;
String logFileName;
bool sdReady = false;

bool setupSDCard(){
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);

    logFileName.reserve(24);
    if (sd.begin(SD_CS_PIN, SPI_SPEED)) {
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
            logFile.close(); // close the file
            Serial.println("Log file created: " + logFileName);
            sdReady = true;
        }else{
            Serial.println(F("SD Card reader found, but file was unable to be created"));
            return false;
        }
    }else{
        Serial.println(F("SD Card Reader NOT found! Data will not be logged!"));
        return false;
    }

    return true;
}