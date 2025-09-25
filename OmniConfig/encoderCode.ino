#include <Arduino.h>

// === Motor pins (PWM + DIR) ===
const int pwmPin[3] = {32, 25, 23}; // M1, M2, M3 PWM
const int dirPin[3] = {33, 26, 22}; // M1, M2, M3 DIR
//********32, 33 motor main********//
// === Encoder pins (A & B) ===
// pick only pins capable of input with interrupts:
const int encA[3] = {34, 35, 36};  // adjust to your wiring
const int encB[3] = {39, 38, 37};  // adjust to your wiring

volatile long encoderCount[3] = {0, 0, 0};

// === LEDC PWM channels ===
const int ch[3] = {0, 1, 2};
const int freq = 20000;
const int resolution = 10; // 0–1023

// === Speed control ===
const int COUNTS_PER_REV = 360; // adjust to your encoder
float targetRPM[3] = {100, 100, 100}; // desired rpm per motor
int pwmValue[3] = {500, 500, 500};    // start PWM (0–1023)

// === Direction map helper ===
enum Motion {STOP=0, FORWARD, BACKWARD, RIGHT, LEFT, CW, CCW};

// attach interrupts
void IRAM_ATTR encoderISR0() {
  int a = digitalRead(encA[0]);
  int b = digitalRead(encB[0]);
  encoderCount[0] += (a == b) ? 1 : -1;
}
void IRAM_ATTR encoderISR1() {
  int a = digitalRead(encA[1]);
  int b = digitalRead(encB[1]);
  encoderCount[1] += (a == b) ? 1 : -1;
}
void IRAM_ATTR encoderISR2() {
  int a = digitalRead(encA[2]);
  int b = digitalRead(encB[2]);
  encoderCount[2] += (a == b) ? 1 : -1;
}

// drive one motor
void driveMotor(int idx, int speed, bool clockwise) {
  digitalWrite(dirPin[idx], clockwise ? HIGH : LOW);
  ledcWrite(ch[idx], speed);
}

void setup() {
  Serial.begin(115200);

  // pins
  for (int i=0; i<3; i++) {
    pinMode(dirPin[i], OUTPUT);
    ledcSetup(ch[i], freq, resolution);
    ledcAttachPin(pwmPin[i], ch[i]);

    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), encoderISR2, CHANGE);

  Serial.println("Enter direction: 1=FWD,2=BACK,3=RIGHT,4=LEFT,5=CW,6=CCW");
}

void setMotion(Motion m) {
  int spd = 800; // base speed

  switch (m) {
    case FORWARD: // m1 clock, m2 none, m3 anticlock
      driveMotor(0, spd, true);
      driveMotor(1, 0, true);
      driveMotor(2, spd, false);
      break;
    case BACKWARD: // m1 anticlock, m2 none, m3 clock
      driveMotor(0, spd, false);
      driveMotor(1, 0, true);
      driveMotor(2, spd, true);
      break;
    case RIGHT: // m1 clock, m2 clock, m3 clock
      driveMotor(0, spd, true);
      driveMotor(1, spd, true);
      driveMotor(2, spd, true);
      break;
    case LEFT: // m1 anticlock, m2 anticlock, m3 anticlock
      driveMotor(0, spd, false);
      driveMotor(1, spd, false);
      driveMotor(2, spd, false);
      break;
    case CW: // m1 clock, m2 anticlock, m3 clock
      driveMotor(0, spd, true);
      driveMotor(1, spd, false);
      driveMotor(2, spd, true);
      break;
    case CCW: // m1 anticlock, m2 clock, m3 anticlock
      driveMotor(0, spd, false);
      driveMotor(1, spd, true);
      driveMotor(2, spd, false);
      break;
    default: // stop all
      for (int i=0; i<3; i++) driveMotor(i, 0, true);
  }
}

void loop() {
  // read direction command
  if (Serial.available()) {
    int d = Serial.parseInt();
    setMotion((Motion)d);
  }

  // every 100ms compute RPM & adjust PWM
  static unsigned long lastTime=0;
  if (millis() - lastTime >= 100) {
    lastTime = millis();

    for (int i=0; i<3; i++) {
      noInterrupts();
      long count = encoderCount[i];
      encoderCount[i]=0;
      interrupts();

      float cps = count * 10.0; // counts per sec
      float rpm = (cps / COUNTS_PER_REV) * 60.0;

      float error = targetRPM[i] - rpm;
      pwmValue[i] += error * 0.5; // proportional
      if (pwmValue[i] > 1023) pwmValue[i] = 1023;
      if (pwmValue[i] < 0) pwmValue[i] = 0;

      // apply new pwm but preserve current direction
      bool dirNow = digitalRead(dirPin[i]);
      driveMotor(i, pwmValue[i], dirNow);

      Serial.print("M"); Serial.print(i);
      Serial.print(" RPM:"); Serial.print(rpm);
      Serial.print(" PWM:"); Serial.print(pwmValue[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
}
