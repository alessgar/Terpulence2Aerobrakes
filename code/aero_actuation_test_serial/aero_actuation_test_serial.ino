#include "wiring_private.h"

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (2ul)               // Pin description number for PIO_SERCOM on D12 -> pin 1 on teensy
#define PIN_SERIAL2_TX       (3ul)               // Pin description number for PIO_SERCOM on D10 -> pin 0 on teensy
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_2)    // SERCOM pad 3

Uart Serial2 (&sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

int i = 90;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Actuation Test Program!"));

  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM);
  Serial2.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  while(!Serial2);
  delay(5000);

  
  //Serial2.println(90.0);
  //delay(5000);
  
}

void loop() {
  /*
  Serial2.println(60.0f);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  Serial2.println(0.0f);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  Serial2.println(60.0f);
  delay(3000);
  Serial2.println(0.0f);
  delay(3000);
  */
  /*
  Serial2.println(60.0f);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10000);
  Serial2.println(0.0f);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
  */
  /*
  Serial2.println(30.0f);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  Serial2.println(60.0f);
  delay(500);
  Serial2.println(85.0f);
  delay(500);
  Serial2.println(0.0f);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  */
  /*
  Serial2.println(90.0f);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  Serial2.println(15.0f);
  delay(50);
  Serial2.println(75.0f);
  delay(50);
  Serial2.println(30.0f);
  delay(50);
  Serial2.println(60.0f);
  delay(50);
  Serial2.println(45.0f);
  delay(50);
  Serial2.println(0.0f);
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);
  */

  /*
  float value = 45.0f + (30.0f * sin(millis()/1000.0f));
  value = ceil(value * 100.0) / 100.0;
  Serial.println(value);
  Serial2.println(value);
  //Serial2.println(10.5f);
  delay(70);
  */

  
  Serial.println(((sin(i) * 30.0) + 45.0));
  Serial2.println(((sin(i++) * 30.0) + 45.0));
  delay(50);

  
  /*
  while(i < 90){
    for(int j = 0; j < 10; j++){
      //Serial.println(i);
      Serial2.println(i);
      delay(50);
    }
    
    i+=1;
    
  }

  while(i >= 0){
     for(int j = 0; j < 10; j++){
      //Serial.println(i);
      Serial2.println(i);
      delay(50);
    }
    i-=1;
  }
  
  */
  

  /*
  Serial2.println(1080);
  delay(3000);
  Serial2.println(0);
  delay(3000);
  Serial2.println(25);
  delay(1000);
  Serial2.println(0);
  delay(1000);
  Serial2.println(50);
  delay(1000);
  Serial2.println(0);
  delay(1000);
  Serial2.println(100);
  delay(1000);
  Serial2.println(0);
  delay(1000);
  Serial2.println(200);
  delay(1000);
  Serial2.println(0);
  delay(1000);
  */
}

void SERCOM2_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}
