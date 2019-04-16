#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>

#define rightMotor 5
#define rightMotorGND 6
#define leftMotor 20
#define leftMotorGND 21

#define XSHUT_pin4 14
#define XSHUT_pin2 15
#define XSHUT_pin1 16

#define Sensor1_newAddress 41
#define Sensor2_newAddress 43
#define Sensor4_newAddress 44
//Timer Interrupts PID Control
IntervalTimer PidTimerLeft;
IntervalTimer PidTimerRight;
//Lidar
VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor4;

bool IsTurn = 0;
long Timeofdetaction = 0;
int  FULL_CIRCLE_PULSE = 374;
float  UNIT_LENGTH = 0.2, SAFETY_RANGE = 0.3, WHEEL_RADIUS = 0.06, radiusOfObstacle, velocity = 0.4;
float distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor;
long PulseCounterRight = 0;
long PulseCounterLeft = 0;
long dt = 0;
long t0 = 0;
long dt2 = 0;
long t02 = 0;
float NeedVelocityRightWheelMS = 0.4;
float NeedVelocityLeftWheelMS = 0.4;
float VelocityRightWheelMS = 0;
float VelocityLeftWheelMS = 0;
float KiLeft = 100;
float KpLeft = 32509;
long t0l;
float KdLeft = 0;
float ErrorLeft;
float PreviousErrorLeft = 0;
float DLeft = 0;
float PLeft = 0;
double ILeft = 0 ;
int ULeft = 0;
float KiRight = 100;
float KpRight = 32509;
long t0r = 0;
float KdRight = 0;
float ErrorRight = 0;
float PreviousErrorRight = 0;
float DRight = 0;
float PRight = 0;
double IRight = 0;
int URight = 0;
long ts_0 = 0;


void setup() {

  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  // comunication
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI.begin();
  Wire.begin();
  Wire.setClock(3000000);
  //Change address of sensor and power up next one
  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);
  Sensor1.setAddress(Sensor1_newAddress);
  delay(20);
  Sensor1.init();
  Sensor2.init();
  Sensor4.init();

  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor4.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor4.startContinuous();
  //Interrupts-Encoder
  attachInterrupt(digitalPinToInterrupt(7), en1_1 , FALLING);
  attachInterrupt(digitalPinToInterrupt(25), en2_1 , RISING);
  //PWM Frequency
  analogWriteFrequency(rightMotor, 5000);
  analogWriteFrequency(rightMotorGND, 5000);
  analogWriteFrequency(leftMotor, 5000);
  analogWriteFrequency(leftMotorGND, 5000);
  //  Duty-Cycle Resolution
  analogWriteResolution(13);

  //PWM legs
  pinMode(rightMotor, OUTPUT);
  pinMode(rightMotorGND, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(leftMotorGND, OUTPUT);
  analogWrite(rightMotor, LOW);
  analogWrite(rightMotorGND, LOW);
  analogWrite(leftMotor, LOW);
  analogWrite(leftMotorGND, LOW);

  //  Encoder legs
  pinMode(8, INPUT);
  pinMode(32, INPUT);

  radiusOfObstacle = 0.1;
  Timeofdetaction = millis();
  //PID Interrupts
  PidTimerLeft.begin(PidLeft, 1000); //1 millis
  PidTimerRight.begin(PidRight, 1000);
}

//Nevigation
void loop()
{
  if (millis() - Timeofdetaction > 5)
  { distanceFromObstacleOfCenterSensor = Sensor2.readRangeContinuousMillimeters();
    if (distanceFromObstacleOfCenterSensor > (UNIT_LENGTH + SAFETY_RANGE) * 1000)
    {
      keepMovingAhead();
      IsTurn = 0;
    }
    else
    {
      distanceFromObstacleOfRightSensor = Sensor1.readRangeContinuousMillimeters();
      distanceFromObstacleOfLeftSensor = Sensor4.readRangeContinuousMillimeters();
      if (!IsTurn)
        turn();
    }
    Timeofdetaction = millis();
  }

 
}


void PidLeft()
{
  ErrorLeft = NeededVelocityLeftWheelMS - VelocityLeftWheelMS;
  ILeft = ILeft + ErrorLeft * 0.001;
  DLeft = KdLeft * (ErrorLeft - PreviousErrorLeft) / 0.001;
  PLeft = KpLeft * ErrorLeft;
  ULeft= PLeft +  DLeft  + KiLeft * ILeft ;
  if (ULeft > 0)
  { if (ULeft > 8191)
      ULeft = 8191;
    analogWrite(leftMotorGND, LOW);
    analogWrite(leftMotor, (int)ULeft);
  }
  if (ULeft<= 0)
  {
    ULeft= -ULeft;
    if (ULeft> 8191)
    ULeft= 8191;
    analogWrite(leftMotorGND, (int)ULeft);
    analogWrite(leftMotor, LOW);
  }
  PreviousErrorLeft = ErrorLeft;

}

void PidRight()
{
  ErrorRight = NeededVelocityRightWheelMS - VelocityRightWheelMS;
  IRight = IRight + ErrorRight * 0.001;
  DRight = KdRight * (ErrorRight - PreviousErrorRight) / 0.001;
  PRight = KpRight * ErrorRight;
  URight =  PRight + DRight  + KiRight * IRight  ;

  if (URight > 0)
  {
    if (URight > 8191)
      URight = 8191;
    analogWrite(rightMotor, (int)URight);
    analogWrite(rightMotorGND, LOW);
  }
  if (URight <= 0)
  { URight = -URight;
    if (URight > 8191)
      URight = 8191;
    analogWrite(rightMotor, LOW);
    analogWrite(rightMotorGND, (int)URight);
  }
  PreviousErrorRight = ErrorRight;
}


void en1_1() {
  dt =  micros() - t0;
  //bool flag = digitalRead(4);
  t0 = micros();
  if (!digitalRead(8))
  { PulseCounterRight++;
    VelocityRightWheelMS = 1567.9963333424992990544387652732 / dt;

  }
  if (digitalRead(8))
  { PulseCounterRight--;
    VelocityRightWheelMS = -1567.9963333424992990544387652732 / dt;

  }

}


void en2_1() {
  dt2 =  micros() - t02;
  t02 = micros();
  if (!digitalRead(32))
  { PulseCounterLeft++;

    VelocityLeftWheelMS = 1567.9963333424992990544387652732 / dt2;

  }
  if (digitalRead(32))

  { PulseCounterLeft--;
    VelocityLeftWheelMS = -1567.9963333424992990544387652732 / dt2;

  }
}

void keepMovingAhead()
{
  VelocityRightWheelMS = velocity;
  VelocityLeftWheelMS = velocity;
}

void turn()
{
  if (distanceFromObstacleOfRightSensor > distanceFromObstacleOfLeftSensor)
    VelocityRightWheelMS = velocity + 0.1;
  VelocityLeftWheelMS = velocity - 0.1;
  if (distanceFromObstacleOfRightSensor = < distanceFromObstacleOfLeftSensor)
    VelocityLeftWheelMS = velocity + 0.1;
  VelocityRightWheelMS = velocity - 0.1;
  IsTurn = 1;
}

