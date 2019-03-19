

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("reciver ready");

}
int k = 0;
char p;
  float velomax = -900;
  float velomin = 900;
void loop() {
  double velo = 0;

  long steps = 0;
  int c = 0;
  int velo_n = 200;

  if (Serial.available()) {
    c = Serial.parseInt();
    Serial2.print(c);
  }
  /* if ( Serial2.available()) {
     c = Serial2.parseInt();
     if (k == 3)
     { Serial.println(c);
       k = 0;
     }
     else
     {
       Serial.print(c);
       k = k + 1;
     Serial.print(' ');
     }
  */
  if ( Serial2.available()) {
    p = Serial2.read();

    if (p == 'l')
    { /*Serial.print("L  ");*/
      /*steps = Serial2.parseInt();
        Serial.print(steps*0.0002519,4);
        dc = Serial2.parseFloat();
        Serial.print(dc,3);
        Serial.print(' ');*/
      while (!Serial2.available());
      velo = Serial2.parseFloat();
      if (velo > velomax)
        velomax = velo;
      
      if (velo < velomin)
          velomin = velo;
      
      Serial.print(((velomax - velomin) / 2), 4);
      Serial.print('\n');
      //Serial.println(millis());
    }
    /*       if (p == 'r')
      {   Serial.print("R  ");
        while (!Serial2.available());
        steps = Serial2.parseInt();
      Serial.print(steps*0.0002519,4);
        Serial.print(' ');
        while (!Serial2.available());
         velo = Serial2.parseFloat();
        Serial.println(velo,6);

      }*/
  }
}
