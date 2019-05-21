#include <SPI.h>
#include "RF24.h"

RF24 radio(4,3);
int dataReceived;
int nodeNum = 1; //need to bo 1-5
uint8_t addresses[][6] = {"1Node","2Node"};
int flag =1;
long t0;
void setup() {

  Serial.begin(115200);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openReadingPipe(1, addresses[nodeNum-1]);
  radio.startListening();
  Serial.println(1);
    Serial.println("reciver..hiii");

  pinMode(7, OUTPUT);

}

void loop() {
     // Serial.println("reciver..hiii");

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
  if(dataReceived==100&&flag){
    flag = 0;
    t0=millis();
}
  if(millis()-t0<5000 &&flag==0){
    digitalWrite(7,HIGH);
  }else
  digitalWrite(7,LOW);

  
}
