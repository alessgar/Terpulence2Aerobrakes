#ifndef AERO_SDCARD
#define AERO_SDCARD

#include "SdFat.h"
#include "sdios.h"

#define sd_FAT_TYPE 0                       // Used to select the storage type we want to use, 0 just means FAT16/FAT32, 1 is FAT16/FAT32, 2 is exFat, and 3 is both
#define SPI_SPEED SD_SCK_MHZ(50)            // How fast should the SD Card writing be? Slow down to 10-20 for breadboards, otherwise 50 when soldered
#define SD_CS_PIN 10                        // What pin is used for CS? Used for SD Card

extern SdFat sd;
extern File logFile;
extern String logFileName;
extern bool sdReady;

#endif