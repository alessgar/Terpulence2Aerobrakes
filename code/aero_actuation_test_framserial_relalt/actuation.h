#ifndef AERO_ACTUATION
#define AERO_ACTUATION

#include <Arduino.h>
#include "wiring_private.h"
#include "fram.h"

// Setup for Teensy Serial
#define PIN_SERIAL2_RX       (2ul)               // Pin description number for PIO_SERCOM on D12 -> pin 1 on teensy
#define PIN_SERIAL2_TX       (3ul)               // Pin description number for PIO_SERCOM on D10 -> pin 0 on teensy
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_2)    // SERCOM pad 3
extern Uart Serial2;                                // Sensor

extern bool hasActuated;                // Used to enforce single time actuation
extern float desiredActuation;          // Used to track desired actuation state
extern bool isActuating;                // Used to track whether we are currently actuating

void outputActuation();                 // Adds actuation data to CSV row
void rotateFlaps();                     // Output our desired actuation to the teensy
void SERCOM2_Handler();                 // Handler for Teensy Serial

bool getHasActuated();                  // Return whether we have actuated
bool getDesiredActuation();             // Returns our desired actuation
bool getIsActuating();                  // Returns whether we are currently actuating
void setHasActuated(bool input);        // Sets the state of whether we have actuated
void setDesiredActuation(float input);  // Sets our desired actuation
void setIsActuating(bool input);        // Sets the state of whether we are current actuating

void setupTeensySerial();               // Setup the Serial connection between us and the Teensy

#endif
