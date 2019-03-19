#include <Wire.h>
#include <VL53L0X.h>

#define rightMotor 2
#define rightMotorGND 3
#define leftMotor 9
#define leftMotorGND 10 

#define XSHUT_pin4 8
#define XSHUT_pin2 6
#define XSHUT_pin1 5

#define Sensor2_newAddress 42
#define Sensor4_newAddress 44

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor4;

int numberOfPulsesOfTheRightEncoder, numberOfPulsesOfTheLeftEncoder, FULL_CIRCLE_PULSE = 748;
float rightMotorSpeed, leftMotorSpeed, UNIT_LENGTH = 0.2, angle, WHEEL_RADIUS = 0.0575;
float distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor;

void setup() {
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  
  Serial.begin(9600);
  Wire.begin();

  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);

  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);

  pinMode(XSHUT_pin1, INPUT);
  delay(10);

  Sensor1.init();
  Sensor2.init();
  Sensor4.init();

  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor4.setTimeout(500);

  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor4.startContinuous();
}

void keepMovingAhead() {
  rightMotorSpeed = 255;
  leftMotorSpeed = 255;
}

void turnRight(float rangeMotionToTheRight, float rangeMotionToTheLeft) {
  if(rangeMotionToTheLeft < 50) {
    rightMotorSpeed = 0;
    leftMotorSpeed = 255;
  }
  else {
    keepMovingAhead();
    //angle = ((numberOfPulsesOfTheLeftEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH)) - ((numberOfPulsesOfTheRightEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH));
  }
}

void turnLeft(float rangeMotionToTheRight, float rangeMotionToTheLeft) {
  if(rangeMotionToTheRight < 50) {
    rightMotorSpeed = 255;
    leftMotorSpeed = 0;
  }
  else {
    keepMovingAhead();
    //angle = ((numberOfPulsesOfTheRightEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH)) - ((numberOfPulsesOfTheLeftEncoder * WHEEL_RADIUS * 360. / FULL_CIRCLE_PULSE) * (360. / UNIT_LENGTH));
  }
}

void navigationOfTheHead(float distanceFromObstacle, float rangeMotionToTheRight, float rangeMotionToTheLeft) {
  if(distanceFromObstacle > 200) {
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
  analogWrite(rightMotorGND, 0);
  analogWrite(leftMotor, leftMotorSpeed);
  analogWrite(leftMotorGND, 0);
}

void loop() {
  Serial.print('Center = ');
  Serial.print(Sensor1.readRangeContinuousMillimeters());
  Serial.print(',');
  Serial.print('Left = ');
  Serial.print(Sensor2.readRangeContinuousMillimeters());
  Serial.print(',');
  Serial.print('Right = ');
  Serial.print(Sensor4.readRangeContinuousMillimeters());
  Serial.print(',');
  Serial.println();
  distanceFromObstacleOfCenterSensor = Sensor1.readRangeContinuousMillimeters();
  distanceFromObstacleOfLeftSensor = Sensor2.readRangeContinuousMillimeters();
  distanceFromObstacleOfRightSensor = Sensor4.readRangeContinuousMillimeters();
  
  navigationOfTheHead(distanceFromObstacleOfCenterSensor, distanceFromObstacleOfRightSensor, distanceFromObstacleOfLeftSensor);
}
