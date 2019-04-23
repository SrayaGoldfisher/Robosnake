#include <Queue.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#define UnitNumber 1

#define rightMotor 5
#define rightMotorGND 6
#define leftMotor 20
#define leftMotorGND 21

//Timer Interrupts PID Control
IntervalTimer PidTimerLeft;
IntervalTimer PidTimerRight;
//Timer Interrupts updating velocity
IntervalTimer  VelocityChangeTimer;
int  FULL_CIRCLE_PULSE = 374;
float  UNIT_LENGTH = 0.2, SAFETY_RANGE = 0.3, WHEEL_RADIUS = 0.06, radiusOfObstacle, velocity = 0.4;
float distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor;
long PulseCounterRight = 0;
long PulseCounterLeft = 0;
long Timeofdetaction;
long dt = 0;
long t0 = 0;
long dt2 = 0;
long t02 = 0;
float NeededVelocityRightWheelMS = 0.4;
float NeededVelocityLeftWheelMS = 0.4;
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
char SerialIndex = ' ';
Queue<float> LeftVelocityqueue = Queue<float>(UnitNumber * 100);
  Queue<float> RightVelocityqueue = Queue<float>(UnitNumber * 100);

void setup() {
//  fill the queue in the start
  int i=0;
  while (i<(UnitNumber * 100-2))
  {
    LeftVelocityqueue.push(0);
    RightVelocityqueue.push(0);
  }

  // comunication
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI.begin();
  Wire.begin();
  Wire.setClock(3000000);

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
  VelocityChangeTimer.begin(VelocityChange, 10000);
}
void loop()
{
  if (Serial.available())
    SerialIndex = Serial.read();

  if (SerialIndex == 'l')
    LeftVelocityqueue.push(Serial.parseFloat());
    if (SerialIndex == 'r')
    RightVelocityqueue.push(Serial.parseFloat());

}

void VelocityChange()
{
  NeededVelocityLeftWheelMS = LeftVelocityqueue.pop();
  NeededVelocityRightWheelMS = RightVelocityqueue.pop();
}
void PidLeft()
{
  ErrorLeft = NeededVelocityLeftWheelMS - VelocityLeftWheelMS;
  ILeft = ILeft + ErrorLeft * 0.001;
  DLeft = KdLeft * (ErrorLeft - PreviousErrorLeft) / 0.001;
  PLeft = KpLeft * ErrorLeft;
  ULeft = PLeft +  DLeft  + KiLeft * ILeft ;
  if (ULeft > 0)
  { if (ULeft > 8191)
      ULeft = 8191;
    analogWrite(leftMotorGND, LOW);
    analogWrite(leftMotor, (int)ULeft);
  }
  if (ULeft <= 0)
  {
    ULeft = -ULeft;
    if (ULeft > 8191)
      ULeft = 8191;
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

