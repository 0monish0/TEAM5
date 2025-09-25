// === Pin assignments for ESP32 + Cytron ===
const int pwmPin[3] = {32, 25, 23}; // M1, M2, M3 PWM
const int dirPin[3] = {33, 26, 22}; // M1, M2, M3 DIR

// === PWM settings ===
const int freq = 20000;     // 20 kHz
const int resolution = 10;  // 0–1023
int pwmValue[3] = {500, 500, 500}; // starting PWM for all motors

// === Setup ===
void setup() {
  Serial.begin(115200);
  Serial.println("Enter direction: 1-forward 2-backward 3-right 4-left 5-clockwise 6-anticlockwise 0-stop");

  // Attach PWM channels and configure pins
  for (int i = 0; i < 3; i++) {
    ledcSetup(i, freq, resolution);
    ledcAttachPin(pwmPin[i], i);   // each motor = its own channel
    pinMode(dirPin[i], OUTPUT);
  }
}

// === Main loop ===
void loop() {
  if (Serial.available() > 0) {
    int dir = Serial.parseInt();
    movement(dir);
  }
}

// === Helper: set motor direction and speed ===
void setMotor(int motor, bool dir, int pwm) {
  digitalWrite(dirPin[motor], dir);  // HIGH = one direction, LOW = other
  ledcWrite(motor, pwm);             // 0–1023
}

// === Movement patterns (your requested directions) ===
void movement(int dir) {
  switch (dir) {
    case 1: // forward: m1 CW, m2 off, m3 CCW
      setMotor(0, HIGH, pwmValue[0]); // M1 CW
      setMotor(1, LOW, 0);            // M2 off
      setMotor(2, LOW, pwmValue[2]);  // M3 CCW
      break;

    case 2: // backward: m1 CCW, m2 off, m3 CW
      setMotor(0, LOW, pwmValue[0]);  // M1 CCW
      setMotor(1, LOW, 0);            // M2 off
      setMotor(2, HIGH, pwmValue[2]); // M3 CW
      break;

    case 3: // right: m1 CW, m2 CW, m3 CW
      setMotor(0, HIGH, pwmValue[0]);
      setMotor(1, HIGH, pwmValue[1]);
      setMotor(2, HIGH, pwmValue[2]);
      break;

    case 4: // left: m1 CCW, m2 CCW, m3 CCW
      setMotor(0, LOW, pwmValue[0]);
      setMotor(1, LOW, pwmValue[1]);
      setMotor(2, LOW, pwmValue[2]);
      break;

    case 5: // clockwise: m1 CW, m2 CCW, m3 CW
      setMotor(0, HIGH, pwmValue[0]);
      setMotor(1, LOW, pwmValue[1]);
      setMotor(2, HIGH, pwmValue[2]);
      break;

    case 6: // anticlockwise: m1 CCW, m2 CW, m3 CCW
      setMotor(0, LOW, pwmValue[0]);
      setMotor(1, HIGH, pwmValue[1]);
      setMotor(2, LOW, pwmValue[2]);
      break;

    case 0: // stop all
    default:
      setMotor(0, HIGH, 0);
      setMotor(1, HIGH, 0);
      setMotor(2, HIGH, 0);
      break;
  }
}
