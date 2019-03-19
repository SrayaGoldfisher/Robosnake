#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X loxCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();

int XSHUTPin1 = 12;
int XSHUTPin2 = 13;
int XSHUTPin3 = 14;

#define rightMotor 9
#define leftMotor 10

int numberOfPulsesOfTheRightEncoder, numberOfPulsesOfTheLeftEncoder, FULL_CIRCLE_PULSE = 748;
float rightMotorSpeed, leftMotorSpeed, UNIT_LENGTH = 0.2, angle, WHEEL_RADIUS = 0.0575;
float distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor;

void setup() {
  pinMode(XSHUTPin1, OUTPUT);
  pinMode(XSHUTPin2, OUTPUT);
  pinMode(XSHUTPin3, OUTPUT);
  digitalWrite(XSHUTPin1, LOW);
  digitalWrite(XSHUTPin2, LOW);
  digitalWrite(XSHUTPin3, LOW);
  delay(500);
  
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  Serial.begin(115200);
  while (! Serial) {
    delay(1);
  }
  if (!loxCenter.begin()) {
    Serial.println(F("Failed to boot VL53L0X_CENTER"));
    while(1);
  }
  if (!loxRight.begin()) {
    Serial.println(F("Failed to boot VL53L0X_RIGRT"));
    while(1);
  }
  if (!loxLeft.begin()) {
    Serial.println(F("Failed to boot VL53L0X_LEFT"));
    while(1);
  }
}

void keepMovingAhead() {
  rightMotorSpeed = 255;
  leftMotorSpeed = 255;
}

void turnRight(float rangeMotionToTheRight, float rangeMotionToTheLeft) {
  if(rangeMotionToTheLeft < 10) {
    rightMotorSpeed = 0;
    leftMotorSpeed = 255;
  }
  else {
    keepMovingAhead();
    //angle = ((numberOfPulsesOfTheLeftEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH)) - ((numberOfPulsesOfTheRightEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH));
  }
}

void turnLeft(float rangeMotionToTheRight, float rangeMotionToTheLeft) {
  if(rangeMotionToTheRight < 10) {
    rightMotorSpeed = 255;
    leftMotorSpeed = 0;
  }
  else {
    keepMovingAhead();
    //angle = ((numberOfPulsesOfTheRightEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH)) - ((numberOfPulsesOfTheLeftEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH));
  }
}

void navigationOfTheHead(float distanceFromObstacle, float rangeMotionToTheRight, float rangeMotionToTheLeft) {
  if(distanceFromObstacle > 150) {
    keepMovingAhead();
  }
  else {
    if(rangeMotionToTheRight >= rangeMotionToTheLeft) {
     // turnRight();
    }
    else {
    //  turnLeft();
    }
  }
  analogWrite(rightMotor, rightMotorSpeed);
  analogWrite(leftMotor, leftMotorSpeed);
}

void loop() {
  digitalWrite(XSHUTPin1, HIGH);
  delay(150);
  VL53L0X_RangingMeasurementData_t measure;
  loxCenter.rangingTest(&measure, false);
  distanceFromObstacleOfCenterSensor = measure.RangeMilliMeter;

  loxRight.setAddress(0x30);
  VL53L0X_RangingMeasurementData_t measure;
  digitalWrite(XSHUTPin2, HIGH);
  delay(150);
  distanceFromObstacleOfRightSensor = measure.RangeMilliMeter;
  
  loxLeft.setAddress(0x31);
  VL53L0X_RangingMeasurementData_t measure;
  digitalWrite(XSHUTPin3, HIGH);
  delay(150);
  distanceFromObstacleOfLeftSensor = measure.RangeMilliMeter;
  
  navigationOfTheHead(distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor);
}
