#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#include "MPU9250.h"
#include "EEPROM.h"

#define rightMotor 5
#define rightMotorGND 6
#define leftMotor 20
#define leftMotorGND 21
MPU9250 Imu(Wire, 0x68);
int status;

// EEPROM buffer and variables to load accel and mag bias
// and scale factors from CalibrateMPU9250.ino
uint8_t EepromBuffer[48];
float axb, axs, ayb, ays, azb, azs;
float hxb, hxs, hyb, hys, hzb, hzs;
#define XSHUT_pin4 14
#define XSHUT_pin2 15
#define XSHUT_pin1 16

#define Sensor1_newAddress 41
#define Sensor2_newAddress 43
#define Sensor4_newAddress 44
//Timer Interrupts PID Control
IntervalTimer PidTimerLeft;
IntervalTimer PidTimerRight;
IntervalTimer PidTimerAngle;

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
float NeededVelocityRightWheelMS = 0.5;
float NeededVelocityLeftWheelMS = 0.5;
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
float StartAngle = 0;
float Angle = 0;
float KiAngle = 10;
float KpAngle = 45;
float KDAngle = 5 ;
float ErrorAngle = 0;
float PreviousErrorAngle = 0;
float DAngle = 0;
float PAngle = 0;
double IAngle = 0;
int Uheading = 0;
char SerialIndex = ' ';
bool begining = 1;
int RealAngle = 0;
int RealPitch = 0;
int StartYaw = 0;
int StartPitch = 0;
float NeededAngle = 0;

//gyro read
double dty = 0;
double t0y = 0;
float omegaYaw = 0;
float thetaYaw = 0;
float omegaP = 0;
float thetaP = 0;
float omegaR = 0;
float thetaR = 0;
float omegaZ = 0;
float omegaY = 0;
float omegaX = 0;
float thetaZ = 0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  Wire.begin();
  Wire.setClock(3000000);

  //Interrupts-Encoder
  attachInterrupt(digitalPinToInterrupt(7), en1_1 , RISING);
  attachInterrupt(digitalPinToInterrupt(22), en2_1 , RISING);
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
  // start communication with IMU
  status = Imu.begin();
  // load and set accel and mag bias and scale
  // factors from CalibrateMPU9250.ino
  for (size_t i = 0; i < sizeof(EepromBuffer); i++) {
    EepromBuffer[i] = EEPROM.read(i);
  }
  memcpy(&axb, EepromBuffer + 0, 4);
  memcpy(&axs, EepromBuffer + 4, 4);
  memcpy(&ayb, EepromBuffer + 8, 4);
  memcpy(&ays, EepromBuffer + 12, 4);
  memcpy(&azb, EepromBuffer + 16, 4);
  memcpy(&azs, EepromBuffer + 20, 4);
  memcpy(&hxb, EepromBuffer + 24, 4);
  memcpy(&hxs, EepromBuffer + 28, 4);
  memcpy(&hyb, EepromBuffer + 32, 4);
  memcpy(&hys, EepromBuffer + 36, 4);
  memcpy(&hzb, EepromBuffer + 40, 4);
  memcpy(&hzs, EepromBuffer + 44, 4);

  Imu.setAccelCalX(axb, axs);
  Imu.setAccelCalY(ayb, ays);
  Imu.setAccelCalZ(azb, azs);

  Imu.setMagCalX(hxb, hxs);
  Imu.setMagCalY(hyb, hys);
  Imu.setMagCalZ(hzb, hzs);
  pinMode(1, OUTPUT);

  // setting a 41 Hz DLPF bandwidth
  Imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 9 for a 100 Hz update rate
  Imu.setSrd(9);
  // enabling the data ready interrupt
  Imu.enableDataReadyInterrupt();
  // attaching the interrupt to microcontroller pin 1
  pinMode(9, INPUT);
  attachInterrupt(9, yaw, RISING);
  //  Encoder legs
  pinMode(8, INPUT);
  pinMode(23, INPUT);
  Timeofdetaction = millis();
  //PID Interrupts
  PidTimerLeft.begin(PidLeft, 1000); //1 millis
  PidTimerRight.begin(PidRight, 1000);
  PidTimerAngle.begin(PidAngle, 1000);
  Serial.println("start");
  Serial1.println("start");

}
long TimeRun = millis();

void loop()
{
  if (millis() - Timeofdetaction > 30) {
    Timeofdetaction = millis();
    Serial.print(ErrorAngle);
    Serial.print("  ");
    //  Serial.print(TimeRun);
    //  Serial.print("  ");
    //  Serial.print(VelocityLeftWheelMS);//NeededVelocityRightWheelMS);
    //  Serial.print("  ");
    Serial.println(Uheading);//NeededVelocityLeftWheelMS);
  }
  while ((millis() - TimeRun) <= 10000)
  {
    NeededVelocityLeftWheelMS = 0.5;
    NeededVelocityRightWheelMS = 0.5;
    //Serial.print(NeededVelocityRightWheelMS);
    // Serial.println(NeededVelocityLeftWheelMS);
    Serial.print(ErrorAngle);
    Serial.print("  ");
    Serial.println(Uheading);
  }
  while ((millis() - TimeRun) <= 20000)
  { Serial.print(ErrorAngle);
    Serial.print("  ");
    Serial.println(Uheading);

    NeededAngle = 90;
  }
  while ((millis() - TimeRun) <= 30000)
  { Serial.print(ErrorAngle);
    Serial.print("  ");
    Serial.println(Uheading);

    NeededAngle = 0;
  }
  NeededVelocityLeftWheelMS = 0;
  NeededVelocityRightWheelMS = 0;
}
void PidLeft()
{
  ErrorLeft = NeededVelocityLeftWheelMS - VelocityLeftWheelMS;
  ILeft = (ILeft + ErrorLeft * 0.001) * (NeededVelocityLeftWheelMS != 0);
  DLeft = KdLeft * (ErrorLeft - PreviousErrorLeft) / 0.001;
  PLeft = KpLeft * ErrorLeft;
  ULeft = PLeft +  DLeft + KiLeft * ILeft + Uheading ;
  if (ULeft >= 0)
  { if (ULeft > 8191)
      ULeft = 8191;
    analogWrite(leftMotorGND, LOW);
    analogWrite(leftMotor, (int)ULeft);
  }
  if (ULeft < 0)
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
  IRight = (IRight + ErrorRight * 0.001) * (NeededVelocityRightWheelMS != 0);
  DRight = KdRight * (ErrorRight - PreviousErrorRight) / 0.001;
  PRight = KpRight * ErrorRight;
  URight =  PRight + DRight  + KiRight * IRight - Uheading  ;

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
void PidAngle()
{ //(int)theta//RealAngle//
  ErrorAngle = NeededAngle - RealAngle;
  IAngle = (IAngle + ErrorAngle * 0.001) * (ErrorAngle != 0);
  DAngle = KDAngle * (ErrorAngle - PreviousErrorAngle) / 0.001;
  PAngle = KpAngle * ErrorAngle;
  Uheading =  PAngle + DAngle  + KiAngle * IAngle  ;

  PreviousErrorAngle = ErrorAngle;

}

void en1_1() {
  dt =  micros() - t0;
  //bool flag = digitalRead(4);
  t0 = micros();
  if (!digitalRead(8))
  { PulseCounterRight++;
    VelocityRightWheelMS = -1567.9963333424992990544387652732 / dt;

  }
  if (digitalRead(8))
  { PulseCounterRight--;
    VelocityRightWheelMS = 1567.9963333424992990544387652732 / dt;

  }

}


void en2_1() {
  dt2 =  micros() - t02;
  t02 = micros();
  if (!digitalRead(23))
  { PulseCounterLeft++;

    VelocityLeftWheelMS = -1567.9963333424992990544387652732 / dt2;

  }
  if (digitalRead(23))

  { PulseCounterLeft--;
    VelocityLeftWheelMS = 1567.9963333424992990544387652732 / dt2;

  }
}
void yaw()
{ Imu.readSensor();
  dty =  micros() - t0y;
  t0y = micros();
  //  begining = 0;
  //local omega
  omegaYaw = Imu.getGyroZ_rads() ;//* (180.0f / PI);
  omegaP = Imu.getGyroX_rads();// * (180.0f / PI);
  omegaR = Imu.getGyroY_rads() ;//* (180.0f / PI);
  thetaP = thetaP + omegaP * (dty / 1000000);
  thetaYaw = thetaYaw + omegaYaw * (dty / 1000000);
  thetaR = thetaR + omegaR * (dty / 1000000);
  //world omega
  omegaZ = (cos(thetaP) * cos(thetaR) * omegaYaw + sin(thetaP) * cos(thetaR) * omegaR - omegaP * sin(thetaR)) * (180.0f / PI);
  omegaX = (cos(thetaYaw) * omegaP - sin(thetaYaw) * omegaR) * (180.0f / PI);
  omegaY = (sin(thetaYaw) * omegaP + cos(thetaYaw) * omegaR) * (180.0f / PI);

  //  thetaY = thetaY + omegaY * (dty / 1000000);
  //   thetaP = thetaP + omegaP * (dty / 1000000);
  //  thetaR = thetaR + omegaR * (dty / 1000000);
  thetaZ = thetaZ + omegaZ * (dty / 1000000);
  RealAngle = thetaZ ;
}
