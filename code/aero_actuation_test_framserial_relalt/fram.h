#ifndef AERO_FRAM
#define AERO_FRAM

#include "Adafruit_FRAM_SPI.h"
#include "sdcard.h"
#include "aerobuzzer.h"

#define FRAM_CS_PIN A4                                       // What pin is used for FRAM? Used for FRAM

extern Adafruit_FRAM_SPI fram;
extern uint32_t framNextLoc;                                    // Next open FRAM location for writing
extern bool isFRAMDumped;                                   // Checked for when FRAM dump condition met
extern bool framReady;

void framPrintln();

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

void framDumpToSD();

#endif