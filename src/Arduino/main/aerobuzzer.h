#ifndef AERO_BUZZER
#define AERO_BUZZER

#include <Buzzer.h>

#define BUZZER_PIN A0                   // What pin is the buzzer inserted into?

extern Buzzer buzz;

void soundBuzz(int totalBeeps);         // Sound the Buzzer

#endif
