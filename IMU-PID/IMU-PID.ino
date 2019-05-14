#include "uNavAHRS.h"
#include "MPU9250.h"
#include "EEPROM.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>

#define UnitNumber 1

#define rightMotor 5
#define rightMotorGND 6
#define leftMotor 20
#define leftMotorGND 21

// an MPU-9250 object on SPI bus 0 with chip select 10
MPU9250 Imu(Wire, 0x68);
int status;
// a flag for when the MPU-9250 has new data
volatile int newData;
// EEPROM buffer and variables to load accel and mag bias
// and scale factors from CalibrateMPU9250.ino
uint8_t EepromBuffer[48];
float axb, axs, ayb, ays, azb, azs;
float hxb, hxs, hyb, hys, hzb, hzs;
//Timer Interrupts PID Control
IntervalTimer PidTimerVelocity;
IntervalTimer PidTimerAngle;
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
float NeededAngle = 0.4;
float NeededVelocity = 0.4;
float VelocityRightWheelMS = 0;
float VelocityLeftWheelMS = 0;
float KiVelocity = 100;
float KpVelocity = 32509;
long t0l;
float KdVelocity = 0;
float ErrorVelocity;
float PrevoiusErrorVelocity = 0;
float DVelocity = 0;
float PVelocity = 0;
double IVeliocity = 0 ;
int UVelocity = 0;
int ULeft = 0;
float KiAngle = 10;
float KpAngle = 3250;
long t0r = 0;
float KDAngle = 0;
float ErrorAngle = 0;
float PreviousErrorAngle = 0;
float DAngle = 0;
float PAngle = 0;
double IAngle = 0;
int URight = 0;
int Uheading = 0;
long ts_0 = 0;
char SerialIndex = ' ';
bool begining = 1;
int RealAngle = 0;
int RealPitch = 0;
int StartYaw = 0;
int StartPitch = 0;
void setup() {
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
  attachInterrupt(9, runFilter, RISING);
  // comunication
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI.begin();
  Wire.begin();
  Wire.setClock(3000000);

  //Interrupts-Encoder
  attachInterrupt(digitalPinToInterrupt(7), en1_1 , FALLING);
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

  //  Encoder legs
  pinMode(8, INPUT);
  pinMode(23, INPUT);

  Timeofdetaction = millis();
  //PID Interrupts
  PidTimerVelocity.begin(PidVelocity, 1000); //1 millis
  PidTimerAngle.begin(PidAngle, 1000);
}
long TimeRun=millis();

void loop()
{
 while (millis()-TimeRun<10000)
 {
   NeededVelocity=0.5;
 }
  NeededVelocity=0;

}

    /*
  if (millis() - Timeofdetaction > 10)
  {

      Serial.print(" angle ");
      Serial.print(RealAngle);
      Serial.print(" Error ");
      Serial.print(ErrorVelocity);
      Serial.print(" left ");
      Serial.print(VelocityLeftWheelMS);
      Serial.print(" right ");
      Serial.println(VelocityRightWheelMS);
    
  }
}
*/

void PidVelocity()
{

  ErrorVelocity = NeededVelocity - (VelocityLeftWheelMS + VelocityRightWheelMS) / 2;
  IVeliocity = IVeliocity + ErrorVelocity * 0.001;
  DVelocity = KdVelocity * (ErrorVelocity - PrevoiusErrorVelocity) / 0.001;
  PVelocity = KpVelocity * ErrorVelocity;
  UVelocity = PVelocity +  DVelocity  + KiVelocity * IVeliocity ;
  ULeft = UVelocity + Uheading;
  URight = UVelocity - Uheading;
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
  PrevoiusErrorVelocity = ErrorVelocity;

}

void PidAngle()
{/*
  ErrorAngle = NeededAngle - RealAngle;
  IAngle = IAngle + ErrorAngle * 0.001;
  DAngle = KDAngle * (ErrorAngle - PreviousErrorAngle) / 0.001;
  PAngle = KpAngle * ErrorAngle;
  Uheading =  PAngle + DAngle  + KiAngle * IAngle  ;

  PreviousErrorAngle = ErrorAngle;
*/
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
  if (!digitalRead(23))
  { PulseCounterLeft++;

    VelocityLeftWheelMS = 1567.9963333424992990544387652732 / dt2;

  }
  if (digitalRead(23))

  { PulseCounterLeft--;
    VelocityLeftWheelMS = -1567.9963333424992990544387652732 / dt2;

  }
}
void runFilter()
{ /*
    Imu.readSensor();
    // update the filter
    if (Filter.update(Imu.getGyroX_rads(), Imu.getGyroY_rads(), Imu.getGyroZ_rads(), Imu.getAccelX_mss(), Imu.getAccelY_mss(), Imu.getAccelZ_mss(), Imu.getMagX_uT(), Imu.getMagY_uT(), Imu.getMagZ_uT()))
    {
     if (begining)
     { digitalWrite(2, HIGH);
       StartPitch = Filter.getPitch_rad() * 180.0f / PI;
       StartYaw = Filter.getYaw_rad() * 180.0f / PI;
       begining = 0;
       RealAngle = StartYaw;
       RealPitch = StartPitch;
     }
     else
     {
       RealPitch = Filter.getPitch_rad() * 180.0f / PI;
       RealAngle = Filter.getYaw_rad() * 180.0f / PI;
     }

    }*/
}
