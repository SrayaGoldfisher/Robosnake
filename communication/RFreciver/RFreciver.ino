#include <SPI.h>
#include "RF24.h"

RF24 radio(7, 8);
int dataReceived;
int nodeNum = 1; //need to bo 1-5
uint8_t addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openReadingPipe(1, addresses[nodeNum-1]);
  radio.startListening();
  Serial.println("reciver..");
}

void loop() {
  if ( radio.available())
  {
    while (radio.available())
    {
      radio.read( &dataReceived, sizeof(dataReceived) );
    }
    radio.stopListening();
    Serial.print("Data received = ");
    Serial.println(dataReceived);
    radio.startListening();
  }
}
