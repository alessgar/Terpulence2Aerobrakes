#include "actuation.h"

// Sensor and other related variables
Uart Serial2(&sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
bool hasActuated = false;               // Used to enforce single time actuation
float desiredActuation = 0.0f;          // Used to track desired actuation state
bool isActuating = false;               // Used to track whether we are currently actuating

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

// Return whether we have actuated
bool getHasActuated(){
    return hasActuated;
}

// Returns our desired actuation
bool getDesiredActuation(){
    return desiredActuation;
}

// Returns whether we are currently actuating
bool getIsActuating(){
    return isActuating;
}

// Sets the state of whether we have actuated
void setHasActuated(bool input){
    hasActuated = input;
}

// Sets our desired actuation
void setDesiredActuation(float input){
    desiredActuation = input;
}

// Sets the state of whether we are current actuating
void setIsActuating(bool input){
    isActuating = input;
}

// Setup the Serial connection between us and the Teensy
void setupTeensySerial(){
    // Setup Teensy Comms
    pinPeripheral(2, PIO_SERCOM);
    pinPeripheral(3, PIO_SERCOM);
    Serial2.begin(115200);
    while(!Serial2);
}

// Handler for Teensy Serial
void SERCOM2_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}
