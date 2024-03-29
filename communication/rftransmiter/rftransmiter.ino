#include <SPI.h>
#include "RF24.h"
#define button1 5 //Power
//#define button2 4 //Brightness+
RF24 radio(4,3);
int dataTransmitted;
uint8_t addresses[][6] = {"1Node","2Node"};
int flag = false;


void setup() {
  Serial.begin(115200);
  pinMode(button1, INPUT);
//  pinMode(button2, INPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openWritingPipe(addresses[0]);
  Serial.println(F("transmiter.."));
  radio.stopListening();

}


void loop() {
  if(digitalRead(button1) == HIGH || flag){
    flag = 1;

     radio.openWritingPipe(addresses[0]);
        radio.stopListening();

      dataTransmitted = 100;
      radio.write( &dataTransmitted, sizeof(dataTransmitted) );
      delay(400);
      Serial.println(dataTransmitted);
  }
/*    if (digitalRead(button1) == HIGH) { //POWER
      radio.openWritingPipe(addresses[0]);
        radio.stopListening();

      dataTransmitted = 100;
      radio.write( &dataTransmitted, sizeof(dataTransmitted) );
      delay(400);
      Serial.println(dataTransmitted);
    }

    if (digitalRead(button2) == HIGH) { //BRIGHTNESS+
        radio.openWritingPipe(addresses[1]);
        radio.stopListening();

      dataTransmitted = 101;
      radio.write( &dataTransmitted, sizeof(dataTransmitted) );
      delay(400);
      Serial.println(dataTransmitted);
    }*/


}
