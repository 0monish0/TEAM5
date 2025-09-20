#include <Arduino.h>

float wheelAngles[3] = { 120.0 * DEG_TO_RAD, 240.0 * DEG_TO_RAD, 0.0 * DEG_TO_RAD }; //wheel angles for basic 3-wheel omni drive

const float wheelRadius = 0.0635;   // hard-coded; sambed
const float Ldistance   = 0.22241;
int in[6] = {2, 3, 4, 5, 6, 7};
int en[3] = {8, 9, 10};
int maxSpeed = x; //To be hard-coded. 

void computeWheelSpeeds(float Vx, float Vy, float omega, float L, float r, float w[3]) {
  for (int i = 0; i < 3; i++) {
    float s = sin(wheelAngles[i]);
    float c = cos(wheelAngles[i]);
    w[i] = (-s * Vx + c * Vy + L * omega) / r;  // matrix computation, each element. in rad/s
  }
}

void computeFromAngle(float speed, float angleDeg, float omega, float L, float r, float w[3]) {
  float angleRad = angleDeg * DEG_TO_RAD;
  float Vx = speed * cos(angleRad);
  float Vy = speed * sin(angleRad);
  computeWheelSpeeds(Vx, Vy, omega, L, r, w);
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  float speed  = 0.5;     // m/s [to be taken input]
  float angle  = 45.0;    // deg [to be taken input]
  float omega  = 0.0;     // rad/s [to be taken input]

  float wheelSpeeds[3];
  computeFromAngle(speed, angle, omega, Ldistance, wheelRadius, wheelSpeeds);
  int pwm[3];
  for(int i=0; i<3; i++){
    int dir = (wheelSpeed[i] > 0) ? 1 : -1
    pwm[i] = (wheelSpeed[i]/maxSpeed)*255; //basic line of code to find pwm value according to the speed needed. 
    if(dir>0){
      digitalWrite(in[i], LOW); //assuming this configures to clockwise
      digitalWrite(in[i+1], HIGH);
      anaogWrite(en[i], pwm[i];
    }
    else {
      digitalWrite(in[i+1], LOW); //assuming this configures to anti-clockwise
      digitalWrite(in[i], HIGH);
      anaogWrite(en[i], pwm[i];
    }
  }
  
  delay(500);
}


