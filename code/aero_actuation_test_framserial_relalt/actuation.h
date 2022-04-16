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
extern Uart Serial2;

extern bool oneTimeActuate;             // Should we only actuate once?
extern bool hasActuated;               // Used to enforce single time actuation
extern float desiredActuation;          // Used to track desired actuation state
extern bool isActuating;

void outputActuation();
void rotateFlaps();
void SERCOM2_Handler();

bool isOneTimeActuate();
bool getHasActuated();
bool getDesiredActuation();
bool getIsActuating();
void setHasActuated(bool input);
void setDesiredActuation(float input);
void setIsActuating(bool input);

void setupTeensySerial();

#endif
