#include <Arduino.h>

// === Motor driver pins ===
int in1[3] = {14, 27, 26};   // IN1 for motor1, motor2, motor3 (choose valid ESP32 GPIOs)
int in2[3] = {25, 33, 32};   // IN2
int en[3]  = {12, 13, 15};   // PWM pins
int encA[3] = {4, 16, 17};
int encB[3] = {5, 18, 19};

volatile long encoderCount[3] = {0,0,0}; // updated in ISRs
float targetRPM[3] = {100, 100, 100};    // desired RPM for each motor
int pwmValue[3] = {150,150,150};         // starting PWM values

const int COUNTS_PER_REV = 360;
int dir = 0;

// === encoder ISRs ===
void IRAM_ATTR encoderISR0() {
  int a = digitalRead(encA[0]);
  int b = digitalRead(encB[0]);
  if (a == b) encoderCount[0]++;
  else encoderCount[0]--;
}

void IRAM_ATTR encoderISR1() {
  int a = digitalRead(encA[1]);
  int b = digitalRead(encB[1]);
  if (a == b) encoderCount[1]++;
  else encoderCount[1]--;
}

void IRAM_ATTR encoderISR2() {
  int a = digitalRead(encA[2]);
  int b = digitalRead(encB[2]);
  if (a == b) encoderCount[2]++;
  else encoderCount[2]--;
}

// === setup ===
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
    pinMode(en[i], OUTPUT);

    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), encoderISR2, CHANGE);

  Serial.println("Enter dir 1=FWD 2=BWD 3=RIGHT 4=LEFT 5=CW 6=CCW:");
}

// === loop ===
void loop() {
  if (Serial.available()) {
    dir = Serial.parseInt();
  }

  switch (dir) {
    case 1: // forward
      digitalWrite(in1[0], HIGH);
      digitalWrite(in2[0], LOW);
      analogWrite(en[0], 255); // CLOCKWISE

      digitalWrite(in1[1], LOW);
      digitalWrite(in2[1], LOW);
      analogWrite(en[1], 0); // head motor

      digitalWrite(in1[2], LOW);
      digitalWrite(in2[2], HIGH);
      analogWrite(en[2], 255); // ANTICLOCK
      break;

    case 2: // backward
      digitalWrite(in1[0], LOW);
      digitalWrite(in2[0], HIGH);
      analogWrite(en[0], 255); // ANTICLOCK

      digitalWrite(in1[1], LOW);
      digitalWrite(in2[1], LOW);
      analogWrite(en[1], 0); // head motor

      digitalWrite(in1[2], HIGH);
      digitalWrite(in2[2], LOW);
      analogWrite(en[2], 255); // CLOCK
      break;

    case 3: // right (fill with your logic)
      digitalWrite(in1[0], HIGH);
      digitalWrite(in2[0], LOW);
      analogWrite(en[0], 255);

      digitalWrite(in1[1], HIGH);
      digitalWrite(in2[1], LOW);
      analogWrite(en[1], 255);

      digitalWrite(in1[2], HIGH);
      digitalWrite(in2[2], LOW);
      analogWrite(en[2], 255);
      break;

    case 4: // left
      digitalWrite(in1[0], LOW);
      digitalWrite(in2[0], HIGH);
      analogWrite(en[0], 255);

      digitalWrite(in1[1], LOW);
      digitalWrite(in2[1], HIGH);
      analogWrite(en[1], 255);

      digitalWrite(in1[2], LOW);
      digitalWrite(in2[2], HIGH);
      analogWrite(en[2], 255);
      break;

    case 5: // clockwise
      digitalWrite(in1[0], HIGH);
      digitalWrite(in2[0], LOW);
      analogWrite(en[0], 255);

      digitalWrite(in1[1], LOW);
      digitalWrite(in2[1], HIGH);
      analogWrite(en[1], 0);

      digitalWrite(in1[2], HIGH);
      digitalWrite(in2[2], LOW);
      analogWrite(en[2], 255);
      break;

    case 6: // counter-clockwise
      digitalWrite(in1[0], LOW);
      digitalWrite(in2[0], HIGH);
      analogWrite(en[0], 255);

      digitalWrite(in1[1], HIGH);
      digitalWrite(in2[1], LOW);
      analogWrite(en[1], 0);

      digitalWrite(in1[2], LOW);
      digitalWrite(in2[2], HIGH);
      analogWrite(en[2], 255);
      break;

    default:
      // stop all
      for (int i = 0; i < 3; i++) analogWrite(en[i], 0);
      break;
  }

  // RPM calculation and simple control
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 100) {
    lastTime = millis();
    for (int i = 0; i < 3; i++) {
      noInterrupts();
      long count = encoderCount[i];
      encoderCount[i] = 0;
      interrupts();

      float cps = count * 10.0;
      float rpm = (cps / COUNTS_PER_REV) * 60.0;
      float error = targetRPM[i] - rpm;
      pwmValue[i] += error * 0.5;
      if (pwmValue[i] > 255) pwmValue[i] = 255;
      if (pwmValue[i] < 0) pwmValue[i] = 0;

      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" RPM:");
      Serial.println(rpm);
    }
  }
}
