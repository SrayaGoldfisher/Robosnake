//Units - m.k.s.

const int CHARight = 2;
const int CHBRight = 7;
const int CHALeft = 3;
const int CHBLeft = 8;
int t0, s0 = 0, teta0 = 0, velocity, AngularVelocity, encoderR = 0, encoderL = 0, FullCirclePulse = 748;
float distance, angle, Wheel = 0.0575, UnitLength = 0.28518928457, Gearbox = 1./34;
int NumTurnsR, NumTurnsL, Kp = 10, Ki = 20, Kd = 10, tetaR, e_prevR, e_presR, teta_dR, P_R, I_R, D_R, U_R;
int tetaL, e_prevL, e_presL, teta_dL, P_L, I_L, D_L, U_L;

void setup() {
  Serial.begin(2000000);
  attachInterrupt(0, NumOfPulseRight, CHANGE);
  attachInterrupt(1, NumOfPulseLeft, CHANGE);
  pinMode(2, INPUT);
  pinMode(7, INPUT);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  Serial.println("Enter a number of desirable turns for the right wheel and the left wheel");
  while (Serial.available() == 0);
  NumTurnsR = Serial.parseInt();
  NumTurnsL = Serial.parseInt();
}

void movement(int NumTurnsR, int NumTurnsL) {
  teta_dR = NumTurnsR * 360;
  tetaR = encoderR * Gearbox * 360. / FullCirclePulse;
  e_prevR = e_presR;
  e_presR = teta_dR - tetaR;

  P_R = Kp * e_presR;
  I_R = Ki * e_presR;
  D_R = Kd * (e_presR - e_prevR);
  U_R = P_R + I_R + D_R;
  if (U_R >= 0) {
    U_R = min(U_R, 255);
    analogWrite(5, U_R);
    analogWrite(6, 0);
  }
  else {
    U_R = abs(U_R);
    U_R = min(U_R, 255);
    analogWrite(6, U_R);
    analogWrite(5, 0);
  }

  teta_dL = NumTurnsL * 360;
  tetaL = encoderL * Gearbox * 360. / FullCirclePulse;
  e_prevL = e_presL;
  e_presL = teta_dL - tetaL;

  P_L = Kp * e_presL;
  I_L = Ki * e_presL;
  D_L = Kd * (e_presL - e_prevL);
  U_L = P_L + I_L + D_L;
  if (U_L >= 0) {
    U_L = min(U_L, 255);
    analogWrite(9, U_L);
    analogWrite(10, 0);
  }
  else {
    U_L = abs(U_L);
    U_L = min(U_L, 255);
    analogWrite(10, U_L);
    analogWrite(9, 0);
  }
}

void NumOfPulseRight() {
  if (digitalRead(CHARight)) {
    if (digitalRead(CHBRight)) {
      encoderR++;
    }
    else {
      encoderR--;
    }
  }
  else {
    if (digitalRead(CHBRight)) {
      encoderR--;
    }
    else {
      encoderR++;
    }
  }
}

void NumOfPulseLeft() {
  if (digitalRead(CHALeft)) {
    if (digitalRead(CHBLeft)) {
      encoderL++;
    }
    else {
      encoderL--;
    }
  }
  else {
    if (digitalRead(CHBLeft)) {
      encoderL--;
    }
    else {
      encoderL++;
    }
  }
}

int Position(int WheelR, int WheelL) {               //AddG earbox?
  if (WheelR == WheelL) {
    angle = 0;
    distance = WheelR * 2 * PI * Wheel * 360. / FullCirclePulse;
  }
  if (WheelR < WheelL) {
    angle = ((WheelL * Wheel * 360. / FullCirclePulse) * (360. / UnitLength)) - ((WheelR * Wheel * 360. / FullCirclePulse) * (360. / UnitLength));
    distance = (WheelR * 2 * PI * Wheel * 360. / FullCirclePulse) + (WheelL * 2 * PI * Wheel * 360. / FullCirclePulse);
  }
  if (WheelR > WheelL) {
    angle = ((WheelR * Wheel * 360. / FullCirclePulse) * (360. / UnitLength)) - ((WheelL * Wheel * 360. / FullCirclePulse) * (360. / UnitLength));
    distance = (WheelR * 2 * PI * Wheel * 360. / FullCirclePulse) + (WheelL * 2 * PI * Wheel * 360. / FullCirclePulse);
  }
  return angle;
}

void loop() {
  if (Serial.available()) {
    t0 = millis();
  }
  movement(NumTurnsR, NumTurnsL);
  angle = Position(encoderR, encoderL);
  velocity = (distance - s0) / (1000 * (millis() - t0));
  AngularVelocity = (angle - teta0) / (1000 * (millis() - t0));
  s0 = distance;
  teta0 = angle;
  Serial.println(millis());
  Serial.println(angle);
  Serial.println(distance);
  Serial.println(velocity);
  Serial.println(AngularVelocity);
  Serial.println();
}
