#ifndef AERO_FRAM
#define AERO_FRAM

#include "Adafruit_FRAM_SPI.h"
#include "sdcard.h"
#include "aerobuzzer.h"
#include "timing.h"

#define FRAM_CS_PIN A4                                       // What pin is used for FRAM? Used for FRAM

extern Adafruit_FRAM_SPI fram;
extern uint32_t framNextLoc;                                // Next open FRAM location for writing

void framPrintln();                                         // Add a newline to the file in FRAM
float getCapacity();                                        // % of storage used in FRAM

// Write string and newline to FRAM
template< typename T > void framPrintln( T data ){
  String str = String(data);
  int strLen = str.length();
  fram.writeEnable(true);
  for(int i = 0; i < strLen; i++){
    fram.write8(framNextLoc++, str.charAt(i));
  }
  fram.write8(framNextLoc++, '\n');
  fram.writeEnable(false);
}

// Write string to FRAM
template< typename T > void framPrint( T data ){
  String str = String(data);
  int strLen = str.length();
  fram.writeEnable(true);
  for(int i = 0; i < strLen; i++){
    fram.write8(framNextLoc++, str.charAt(i));
  }
  fram.writeEnable(false);
}

void startRow(float curTime);                               // inserts timestamp to start data row. Argument is timestamp

void endRow();                                              // ends the row and adds a newline

void framDumpToSD();                                        // Dump FRAM to SD Card

bool isFramReady();                                         // Returns whether the FRAM is initialized
bool isFramDumped();                                        // Returns whether the FRAM has been dumped
int getFramNextLoc();                                       // Returns the next free memory location in FRAM
bool setupFram();                                           // Initializes the FRAM
void resetDumpStatus();                                     // Resets FRAM Dump Status

void insertBlankValues(int numValues);                      // Adds the specified number of blank values to the CSV row. Argument is # of blank values to insert
#endif
