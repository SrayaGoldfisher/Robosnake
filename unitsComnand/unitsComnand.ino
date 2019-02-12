//units - m.k.s

float velocity, angularVelocity, _position, UNIT_LENGTH = 0.28518928457, radiusOfCurv, DISTANSE_BETWEEN_UNIT_AND_UNIT = 0.5, startOfCurv, endOfCurv, countRightWheelLoop, countLeftWheelLoop, angularVelocityRightWheel, angularVelocityLeftWheel;
int unitNumber;

void setup() {
  Serial.begin(2000000);
  while (Serial.available() == 0);
  _position = Serial.parseFloat();
  angularVelocity = Serial.parseFloat();
  velocity = Serial.parseFloat();
  countRightWheelLoop = Serial.parseFloat();
  countLeftWheelLoop = Serial.parseFloat();
  unitNumber = Serial.parseInt();
}

void doRightTurn() {
  if(_position == (startOfCurv + DISTANSE_BETWEEN_UNIT_AND_UNIT * unitNumber)) {
    if(_position < endOfCurv) {
      radiusOfCurv = velocity / angularVelocity;
      angularVelocityRightWheel = velocity / radiusOfCurv;
      angularVelocityLeftWheel = velocity / (radiusOfCurv + UNIT_LENGTH);
    }
  }
}

void doLeftTurn() {
  if(_position == (startOfCurv + DISTANSE_BETWEEN_UNIT_AND_UNIT * unitNumber)) {
    if(_position < endOfCurv) {
      radiusOfCurv = velocity / angularVelocity;
      angularVelocityLeftWheel = velocity / radiusOfCurv;
      angularVelocityRightWheel = velocity / (radiusOfCurv + UNIT_LENGTH);
    }
  }
}

void loop() {
  if(countRightWheelLoop == countLeftWheelLoop) {
    angularVelocityRightWheel = velocity;
    angularVelocityLeftWheel = velocity;
  }
  if(countRightWheelLoop > countLeftWheelLoop) {
    startOfCurv = _position;
    doLeftTurn();
  }
  if(countRightWheelLoop < countLeftWheelLoop) {
    startOfCurv = _position;
    doRightTurn();
  }
}
