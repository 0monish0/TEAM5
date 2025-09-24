// Motor driver pins
int in1[3] = {2, 4, 6};  // IN1 for motor1, motor2, motor3
int in2[3] = {3, 5, 7};  // IN2
int en[3]  = {9, 10, 11}; // PWM pins

// Encoder channel A pins (must be interrupt capable on Uno)
int encA[3] = {18, 19, 20}; // if using a Mega; on Uno you only have 2 ints
// Encoder channel B pins
int encB[3] = {22, 23, 24}; // any digital pins

volatile long encoderCount[3] = {0,0,0}; // updated in ISRs
float targetRPM[3] = {100, 100, 100}; // desired RPM for each motor
int pwmValue[3] = {150,150,150}; // starting PWM values

const int COUNTS_PER_REV = 360; // adjust to your encoder spec

//encoder ISR functions
void encoderISR0() {
  int a = digitalRead(encA[0]);
  int b = digitalRead(encB[0]);
  if (a == b) encoderCount[0]++;
  else encoderCount[0]--;
}

void encoderISR1() {
  int a = digitalRead(encA[1]);
  int b = digitalRead(encB[1]);
  if (a == b) encoderCount[1]++;
  else encoderCount[1]--;
}

void encoderISR2() {
  int a = digitalRead(encA[2]);
  int b = digitalRead(encB[2]);
  if (a == b) encoderCount[2]++;
  else encoderCount[2]--;
}

//attach in setup
void setup() {
  Serial.begin(9600);

  for (int i=0;i<3;i++){
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
    pinMode(en[i], OUTPUT);

    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), encoderISR2, CHANGE);
}

//final 
void loop() {
  // set direction (forward example)
  for(int i=0;i<3;i++){
    digitalWrite(in1[i], HIGH);
    digitalWrite(in2[i], LOW);
    analogWrite(en[i], pwmValue[i]);
  }

  static unsigned long lastTime=0;
  if (millis()-lastTime >= 100) {
    lastTime=millis();
    for (int i=0;i<3;i++){
      noInterrupts();
      long count=encoderCount[i];
      encoderCount[i]=0;
      interrupts();

      float cps = count*10.0; // counts per second
      float rpm = (cps/COUNTS_PER_REV)*60.0;

      // simple proportional control (replace with PID for smoother)
      float error = targetRPM[i]-rpm;
      pwmValue[i] += error*0.5; // Kp=0.5 example
      if (pwmValue[i]>255) pwmValue[i]=255;
      if (pwmValue[i]<0) pwmValue[i]=0;

      Serial.print("Motor ");Serial.print(i);Serial.print(" RPM:");
      Serial.println(rpm);
    }
  }
}

