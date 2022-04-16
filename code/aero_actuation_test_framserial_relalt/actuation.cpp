#include "actuation.h"

bool oneTimeActuate = true;             // Should we only actuate once?

Uart Serial2(&sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
bool hasActuated = false;               // Used to enforce single time actuation
float desiredActuation = 0.0f;          // Used to track desired actuation state
bool isActuating = false;

// Adds actuation data to CSV row
void outputActuation() {
  if (isFramReady()) {
    insertBlankValues(1);
    framPrint(desiredActuation);
  } else {
    insertBlankValues(2);
  }
}

// Output our desired actuation to the teensy
void rotateFlaps() {
  Serial2.println(desiredActuation);
}

bool isOneTimeActuate(){
    return oneTimeActuate;
}

bool getHasActuated(){
    return hasActuated;
}

bool getDesiredActuation(){
    return desiredActuation;
}

bool getIsActuating(){
    return isActuating;
}

void setHasActuated(bool input){
    hasActuated = input;
}

void setDesiredActuation(float input){
    desiredActuation = input;
}

void setIsActuating(bool input){
    isActuating = input;
}

void setupTeensySerial(){
    // Setup Teensy Comms
    pinPeripheral(2, PIO_SERCOM);
    pinPeripheral(3, PIO_SERCOM);
    Serial2.begin(115200);
    while(!Serial2);
}

void SERCOM2_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}