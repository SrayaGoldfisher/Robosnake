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
double veloln = 0.5;
double velorn = 0.5;
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
long ts_0 = 0;
char p = ' ';
long change = millis();

//units mm.k.s.

#include <Wire.h>
#include <VL53L0X.h>

#define rightMotor 5
#define rightMotorGND 6
#define leftMotor 9
#define leftMotorGND 10

#define XSHUT_pin4 14
#define XSHUT_pin2 12
#define XSHUT_pin1 11

#define Sensor2_newAddress 42
#define Sensor4_newAddress 44

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor4;

int numberOfPulsesOfTheRightEncoder, numberOfPulsesOfTheLeftEncoder, FULL_CIRCLE_PULSE = 374;
float rightMotorSpeed, leftMotorSpeed, UNIT_LENGTH = 0.2, SAFETY_RANGE = 0.3, angleOfTheTurn, WHEEL_RADIUS = 0.06, radiusOfObstacle, velocity = 0.5;
float distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor;
float LR = 1;
void setup() {
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  Wire.begin();

  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);

  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);

  pinMode(XSHUT_pin1, INPUT);
  delay(10);


  attachInterrupt(digitalPinToInterrupt(7), en1_1 , FALLING);
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
  pinMode(8, INPUT);
  pinMode(32, INPUT);
  analogWrite(5, LOW);
  analogWrite(6, LOW);
  analogWrite(9, LOW);
  analogWrite(10, LOW);

  Sensor1.init();
  Sensor2.init();
  Sensor4.init();

  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor4.setTimeout(500);

  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor4.startContinuous();


  radiusOfObstacle = 0.1;
}

void keepMovingAhead()
{ LR = 1;
  veloln = velocity;
  velorn = velocity;
}

void turn()
{
  if (distanceFromObstacleOfRightSensor > distanceFromObstacleOfLeftSensor)
    LR = 5 / 3;
  if (distanceFromObstacleOfRightSensor < distanceFromObstacleOfLeftSensor)
    LR = 3 / 5;
  veloln = LR * velocity;
  velorn = 1 - veloln;
}

void loop()
{


  

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
      analogWrite(10, LOW);
      analogWrite(9, (int)ul);
    }
    if (ul <= 0)
    {
      ul = -ul;
      if (ul > 8191)
        ul = 8191;
      analogWrite(10, (int)ul);
      analogWrite(9, LOW);
    }
    errorPRl = errorl;
    t0l = millis();
  }
  //right
  if (millis() - t0r > 1)
  {
    kpr = 32509;
    kir = 82;
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
  if (!digitalRead(32))
  { counter_2++;

    velo_l = 1567.9963333424992990544387652732 / dt2;

  }
  if (digitalRead(32))

  { counter_2--;
    velo_l = -1567.9963333424992990544387652732 / dt2;

  }
}
