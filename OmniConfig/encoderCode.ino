// Motor driver pins
int in1[3] = {2, 4, 6};  // IN1 for motor1, motor2, motor3
int in2[3] = {3, 5, 7};  // IN2
int en[3]  = {9, 10, 11}; // PWM pins
int encA[3] = {18, 19, 20};
int encB[3] = {22, 23, 24};

volatile long encoderCount[3] = {0,0,0}; // updated in ISRs
float targetRPM[3] = {100, 100, 100}; // desired RPM for each motor
int pwmValue[3] = {150,150,150}; // starting PWM values

const int COUNTS_PER_REV = 360;
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
  switch (dir){
    case 1: 
        digitalWrite(in1[0], HIGH);
        digitalWrite(in2[0], LOW);
        analogWrite(en[0], 255);

        digitalWrite(in1[1], LOW);
        digitalWrite(in2[1], LOW);
        analogWrite(en[1], 0);

        digitalWrite(in1[2], HIGH);
        digitalWrite(in2[2], LOW);
        analogWrite(en[2], 255);
		break;
    
    case 2: 
        digitalWrite(in1[0], LOW);
        digitalWrite(in2[0], HIGH);
        analogWrite(en[0], 255);

        digitalWrite(in1[1], LOW);
        digitalWrite(in2[1], LOW);
        analogWrite(en[1], 0);

        digitalWrite(in1[2], LOW);
        digitalWrite(in2[2], HIGH);
        analogWrite(en[2], 255);
		break;
    
    case 3: //to code
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, 255*(23/25));

        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0);

        digitalWrite(in5, LOW);
        digitalWrite(in6, HIGH);
        analogWrite(enC, 255);
		break;
    
    case 4: //to code
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, 255*(23/25));

        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0);

        digitalWrite(in5, LOW);
        digitalWrite(in6, HIGH);
        analogWrite(enC, 255);
		break;
    
    case 5:
    	  digitalWrite(in1[0], LOW);
        digitalWrite(in2[0], HIGH);
        analogWrite(en[0], 255);

        digitalWrite(in1[1], LOW);
        digitalWrite(in2[1], HIGH);
        analogWrite(en[1], 0);

        digitalWrite(in1[2], LOW);
        digitalWrite(in2[2], HIGH);
        analogWrite(en[2], 255);
		break;

    case 6:
    	digitalWrite(in1[0], HIGH);
        digitalWrite(in2[0], LOW);
        analogWrite(en[0], 255);

        digitalWrite(in1[1], HIGH);
        digitalWrite(in2[1], LOW);
        analogWrite(en[1], 0);

        digitalWrite(in1[2], HIGH);
        digitalWrite(in2[2], LOW);
        analogWrite(en[2], 255);
}

  static unsigned long lastTime=0;
  if (millis()-lastTime >= 100) {
    lastTime=millis();
    for (int i=0;i<3;i++){
      noInterrupts();
      long count=encoderCount[i];
      encoderCount[i]=0;
      interrupts();

      float cps = count*10.0;
      float rpm = (cps/COUNTS_PER_REV)*60.0;
      float error = targetRPM[i]-rpm;
      pwmValue[i] += error*0.5;
      if (pwmValue[i]>255) pwmValue[i]=255;
      if (pwmValue[i]<0) pwmValue[i]=0;

      Serial.print("Motor ");Serial.print(i);Serial.print(" RPM:");
      Serial.println(rpm);
    }
  }
}

