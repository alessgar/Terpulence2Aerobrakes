#include "aerobuzzer.h"

Buzzer buzz(BUZZER_PIN);

// Sound the Buzzer
void soundBuzz(int totalBeeps){
  buzz.begin(BUZZER_PIN);

  for(int i = 0; i < totalBeeps; i++){
    buzz.sound(NOTE_G3, 250);
    delay(100);
  }
  
  buzz.end(0);
}
