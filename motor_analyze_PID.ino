#include <avr/io.h>
#include <avr/interrupt.h>
long counter_1 = 0;
long counter_2 = 0;
long dt = 0;
long t0 = 0;
long dt2 = 0;
long t02 = 0;
int velo_l = 0;
int velo_r = 0;
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
float Dl;
float Pl;
double Il ;
int ul;
float kir = 0;
float kpr = 0;
long t0r;
float kdr = 0;
float errorr;
float errorPRr = 0;
float Dr;
float Pr;
double Ir ;
int ur;
void setup() {

  attachInterrupt(digitalPinToInterrupt(3), en1_1 , RISING);
  attachInterrupt(digitalPinToInterrupt(25), en2_1 , RISING);
  analogWriteFrequency(5, 5000);
  analogWriteFrequency(6, 5000);
  analogWriteFrequency(9, 5000);
  analogWriteFrequency(10, 5000);
  analogWriteResolution(13);


  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  analogWrite(5, LOW);
  analogWrite(6, LOW);
  analogWrite(9, LOW);
  analogWrite(10, LOW);
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
void loop() {

  if (millis() - ts_0 >= 20)
  {
    ts_0 = millis();
    Serial1.print('l');

    Serial1.print(velo_r);
    Serial1.print('\n');
  }

  kpl = 0.0043;
  kil = 153.66;

  errorl = veloln - velo_l;
  Il = Il + errorl;
  Dl = kdl * (errorl - errorPRl);
  Pl = kpl * errorl;
  ul = Pl + Dl / 10 + kil * Il * 0.000002;
  if (millis() - t0l > 100)
  {
    if (ul > 255)
    { ul = 255;
      analogWrite(9, ul);
      analogWrite(10, LOW);
    }
    else
    {
      ul = -ul;
      if (ul > 255)
        ul = 255;
      analogWrite(9, LOW);
      analogWrite(10, ul);
    }
    errorPRl = errorl;
    t0l = millis();
  }
  errorPRl = errorl;
  //right
  kpr = 0.6852;
  kir = 85.11;
  kdr = 0.00006412;
        errorr = velorn - velo_r;
  Ir = Ir + errorr;
  Dr = kdr * (errorr - errorPRr);
  Pr = kpr * errorr;
  ur = Pr + Dr / 10 + kir * Ir * 0.000002;
  if (millis() - t0r > 100)
  {
    if (ur > 255)
    { ur = 255;
      analogWrite(9, ur);
      analogWrite(10, LOW);
    }
    else
    {
      ur = -ur;
      if (ur > 255)
        ur = 255;
      analogWrite(9, LOW);
      analogWrite(10, ur);
    }
    errorPRl = errorl;
    t0r = millis();
  }
  errorPRr = errorr;

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


}




void en1_1() {
  dt =  micros() - t0;
  t0 = micros();
  if (digitalRead(4) == digitalRead(3))
  { counter_1++;
    velo_r = 2 * 80213.9037 / dt;

  }
  else
  { counter_1--;
    velo_r = -2 * 80213.9037 / dt;

  }

}


void en2_1() {
  dt2 =  micros() - t02;
  t02 = micros();
  if (digitalRead(25) == digitalRead(32))
  { counter_2++;

    velo_l = 2 * 80213.9037 / dt2;

  }
  else
  { counter_2--;
    velo_l = -2 * 80213.9037 / dt2;

  }

}

