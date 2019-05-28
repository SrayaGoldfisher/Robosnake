#include <avr/io.h>
#include <avr/interrupt.h>
long counter_1 = 0;
long counter_2 = 0;
long dt = 0;
long t0 = 0;
long dt2 = 0;
long t02 = 0;
float velo_l = 0;
float velo_r = 0;
long tlop0 = 0;
int k = 0;
String inputString = "";
bool stringComplete = false;
int velocl = 0;
int velocr = 0;
double velo_l_s = 0;
double velo_r_s = 0;
double veloln = 0;
double velorn = 0;
int dtmotor = 0;
int t0motor = 0;
float kil = 0;
float kpl = 0;
long t0l;
float kdl = 0;
float errorl;
float errorPRl = 0;
float Dl = 0;
float Pl = 0;
double Il = 0 ;
int ul = 0;
float kir = 0;
float kpr = 0;
long t0r = 0;
float kdr = 0;
float errorr = 0;
float errorPRr = 0;
float Dr = 0;
float Pr = 0;
double Ir = 0;
int ur = 0;
void setup() {

  attachInterrupt(digitalPinToInterrupt(7), en1_1 , FALLING);  
  attachInterrupt(digitalPinToInterrupt(22), en2_1 , RISING);
  analogWriteFrequency(5, 5000);
  analogWriteFrequency(6, 5000);
  analogWriteFrequency(20, 5000);
  analogWriteFrequency(21, 5000);
  analogWriteResolution(13);


  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(8, INPUT);
  pinMode(23, INPUT);
  analogWrite(5, LOW);
  analogWrite(6, LOW);
  analogWrite(20, LOW);
  analogWrite(21, LOW);
  /* digitalWrite(5, LOW);
    digitalWrite(6, 50);
    digitalWrite(9, 50);
    digitalWrite(10, LOW);
    delay(500);
     digitalWrite(5, LOW);
     digitalWrite(6, 50);
     digitalWrite(9, 50);
     digitalWrite(10, LOW);
     delay(500);
     digitalWrite(5, LOW);
     digitalWrite(6, LOW);
     digitalWrite(9, LOW);
     digitalWrite(10, LOW);
  */
}
long ts_0 = 0;
char p=' ';
long change = millis();

void loop() {
if ( Serial1.available()) {
    p = Serial1.read();

    if (p == 'l')
    {
        while (!Serial1.available());
      veloln = Serial1.parseFloat();
      }
      
    if (p == 'r')
    {
        while (!Serial1.available());
      velorn = Serial1.parseFloat();
      }
    }
  /*if(Serial1.available())
    {  veloln = Serial.parseInt();
    velorn=veloln;}*/
  /*if (millis() - change >= 20000)
  { 
   
    veloln = veloln+0.1;

    
    velorn=velorn-0.1;
  
    change = millis();
  }*/
  if (millis() - ts_0 >= 100)
  {
    ts_0 = millis();
    Serial1.print('l');

    Serial1.print(velo_l);
    Serial1.print('\n');
    Serial1.print('r');

    Serial1.print(velo_r);
    Serial1.print('\n');
  Serial.print('l');

   /* Serial.print(velo_l);

    Serial.print('r');

    Serial.print(velo_r);
 Serial.println('l');
  */
  }
  if (millis() - t0l > 1)
  {
    kpl = 32509;
    kil = 100;
    kdl = 0;
    errorl = veloln - velo_l;
    Il = Il + errorl * 0.001;
    Dl = kdl * (errorl - errorPRl) / 0.001;
    Pl = kpl * errorl;
    ul = Pl +  Dl  + kil * Il ;
    if (ul > 0)
    { if (ul > 8191)
        ul = 8191;
      analogWrite(21, LOW);
      analogWrite(20, (int)ul);
    }
    if (ul <= 0)
    {
      ul = -ul;
      if (ul > 8191)
        ul = 8191;
      analogWrite(21, (int)ul);
      analogWrite(20, LOW);
    }
    errorPRl = errorl;
    t0l = millis();
  }
  //right
  if (millis() - t0r > 1)
  {
    kpr = 32509;
    kir =82;
    kdr = 0;
    errorr = velorn - velo_r;
    Ir = Ir + errorr * 0.001;
    Dr = kdr * (errorr - errorPRr) / 0.001;
    Pr = kpr * errorr;
    ur = (int) Pr + (int)Dr  + (int)kir * (int)Ir  ;

    if (ur > 0)
    {
      if (ur > 8191)
        ur = 8191;
      analogWrite(5, ur);
      analogWrite(6, LOW);
    }
    if (ur <= 0)
    { ur = -ur;
      if (ur > 8191)
        ur = 8191;
      analogWrite(5, LOW);
      analogWrite(6, ur);
    }
    t0r = millis();
    errorPRr = errorr;

  }
  /*
    k = 4691 + 3500 * sin(2 * 3.141592 * millis() * 2.5 * 0.001);

    Serial.print(k);
    if ( (k > 0) && (k < 8191))
    { analogWrite(5, LOW);
      analogWrite(6, k);
      analogWrite(9, LOW);
      analogWrite(10, k);
    }
    if ((k < 0) && (k > -8191))
    {
      analogWrite(5, abs(k));
      analogWrite(6, LOW);
      analogWrite(9, abs(k));
      analogWrite(10, LOW);
    }
    if (k == 0)
    {
      analogWrite(5, LOW);
      analogWrite(6, LOW);
      analogWrite(9, LOW);
      analogWrite(10, LOW);
      velo_r = 0;
      velo_l = 0;
      Il = 0;
    }

  */
}




void en1_1() {
  dt =  micros() - t0;
  //bool flag = digitalRead(4);
  t0 = micros();
  if (!digitalRead(8))
  { counter_1++;
    velo_r = 1567.9963333424992990544387652732 / dt;
    
  }
  if (digitalRead(8))
  { counter_1--;
    velo_r = -1567.9963333424992990544387652732 / dt;
   
  }
  
}


void en2_1() {
  dt2 =  micros() - t02;
  t02 = micros();
  if (!digitalRead(22))
  { counter_2++;

    velo_l = -1567.9963333424992990544387652732 / dt2;

  }
  if (digitalRead(22))

  { counter_2--;
    velo_l = 1567.9963333424992990544387652732 / dt2;

  }
}
