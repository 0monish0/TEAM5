// ESP32 Devkit V1 + Cytron Motor Drivers (PWM + DIR)

// Pins
const int pwmPin[3] = {32, 25, 23}; // PWM for M1, M2, M3
const int dirPin[3] = {33, 26, 22}; // DIR for M1, M2, M3

// LEDC channels (must be unique 0–15)
const int pwmChannel[3] = {0, 1, 2};
const int pwmFreq = 20000;    // 20kHz (silent)
const int pwmResolution = 8;  // 8-bit (0-255)

// Speed values for each motor (0–255)
int speedVal[3] = {255, 255, 255};  // default max

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 OmniBot Ready");
  Serial.println("Enter direction: 1=Fwd, 2=Bwd, 3=Right, 4=Left, 5=CW, 6=CCW");

  // Setup pins + LEDC PWM
  for (int i = 0; i < 3; i++) {
    pinMode(dirPin[i], OUTPUT);
    ledcSetup(pwmChannel[i], pwmFreq, pwmResolution);
    ledcAttachPin(pwmPin[i], pwmChannel[i]);
    ledcWrite(pwmChannel[i], 0); // start off
  }
}

void loop() {
  // wait for serial input
  if (Serial.available()) {
    int dir = Serial.parseInt();
    Serial.print("Command received: ");
    Serial.println(dir);
    moveRobot(dir);
  }
}

// Smooth fade to speedVal
void setMotor(int motor, bool forward, int speed) {
  digitalWrite(dirPin[motor], forward ? HIGH : LOW);
  // smooth ramp to target
  ledcWriteTone(pwmChannel[motor], 0); // ensure no tone
  ledcWrite(pwmChannel[motor], speed); // direct write (instant)
  Serial.print("Motor "); Serial.print(motor);
  Serial.print(forward ? " FWD " : " REV ");
  Serial.print(" speed=");
  Serial.println(speed);
}

void moveRobot(int dir) {
  switch (dir) {
    case 1: // Forward
      setMotor(0, true,  speedVal[0]);
      setMotor(1, false, 0);
      setMotor(2, true,  speedVal[2]);
      break;
    case 2: // Backward
      setMotor(0, false, speedVal[0]);
      setMotor(1, false, 0);
      setMotor(2, false, speedVal[2]);
      break;
    case 3: // Right
      setMotor(0, false, speedVal[0]);
      setMotor(1, false, 0);
      setMotor(2, false, speedVal[2]);
      break;
    case 4: // Left
      setMotor(0, true,  speedVal[0]);
      setMotor(1, false, 0);
      setMotor(2, true,  speedVal[2]);
      break;
    case 5: // Clockwise
      setMotor(0, false, speedVal[0]);
      setMotor(1, true,  speedVal[1]);
      setMotor(2, false, speedVal[2]);
      break;
    case 6: // Counter-Clockwise
      setMotor(0, true,  speedVal[0]);
      setMotor(1, false, speedVal[1]);
      setMotor(2, true,  speedVal[2]);
      break;
    default: // stop
      stopAll();
      break;
  }
}

void stopAll() {
  for (int i = 0; i < 3; i++) {
    ledcWrite(pwmChannel[i], 0);
  }
  Serial.println("Motors stopped");
}
