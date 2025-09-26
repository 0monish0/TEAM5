// This ESP32 program connects to Wi-Fi, starts a WebSocket server, and listens for movement commands.
// When it receives a command, it drives three omni wheels accordingly to move or rotate the robot. The motion for each wheel is simply hard-coded since holonomic motion is not required. 

#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "esp32"; //configured as wifi station; ssid
const char* password = "987654321";

const int pwmPin[3] = {32, 25, 23}; // PWM for M1, M2, M3
const int dirPin[3] = {33, 26, 22}; // DIR for M1, M2, M3

const int pwmFreq = 20000;    
const int pwmResolution = 8;  // 8-bit
int speedVal[3] = {128, 128, 128};  //half for safety

WebSocketsServer webSocket(81); //starting web socket server on port 81

void setMotor(int motor, bool forward, int speed) { 
  digitalWrite(dirPin[motor], forward ? HIGH : LOW);
  ledcWrite(pwmPin[motor], speed);

  Serial.print("Motor ");
  Serial.print(motor);
  Serial.print(forward ? " FWD " : " REV ");
  Serial.print(" Speed=");
  Serial.println(speed);
}

void stopAll() {
  for (int i = 0; i < 3; i++) {
    ledcWrite(pwmPin[i], 0);
  }
  Serial.println("Motors STOPPED");
}

void moveRobot(int dir) {
  Serial.print("Command: ");
  Serial.println(dir);

  switch (dir) {
    case 1: Serial.println("Moving FORWARD");
      setMotor(0, true,  speedVal[0]); //clockwise
      setMotor(1, false, 0); //none
      setMotor(2, false,  speedVal[2]); //anticlockwise
      break;
    case 2: Serial.println("Moving BACKWARD");
      setMotor(0, false, speedVal[0]); //anticlockwise
      setMotor(1, false, 0); //none
      setMotor(2, true,  speedVal[2]); //clockwise
      break;
    case 3: Serial.println("Moving RIGHT");
      setMotor(0, true, speedVal[0]); //clockwise
      setMotor(1, true, speedVal[1]); //clockwise
      setMotor(2, true, speedVal[2]); //clockwise
      break;
    case 4: Serial.println("Moving LEFT");
      setMotor(0, false,  speedVal[0]); //anticlockwise
      setMotor(1, false, speedVal[1]); //anticlockwise
      setMotor(2, false,  speedVal[2]); //anticlockwise
      break;
    case 5: Serial.println("Rotating CLOCKWISE");
      setMotor(0, true,  speedVal[0]); //clockwise
      setMotor(1, false, speedVal[1]); //anticlockwise
      setMotor(2, true,  speedVal[2]); //clockwise
      break;
    case 6: Serial.println("Rotating COUNTER-CLOCKWISE");
      setMotor(0, false,  speedVal[0]); //anticlockwise
      setMotor(1, true, speedVal[1]); //clockwise
      setMotor(2, false, speedVal[2]); //anticlockwise
      break;
    default: 
      Serial.println("STOP command received");
      stopAll();
      break;
  }
}

// ===== WebSocket Event =====
void webSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) { //callback function, type gives the current status
  switch (type) { 
    case WStype_CONNECTED: 
      Serial.printf("Client %u connected\n", client_num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("Client %u disconnected\n", client_num);
      break;
    case WStype_TEXT:
      payload[length] = '\0';
      Serial.printf("From Client %u: %s\n", client_num, payload);
      moveRobot(atoi((char*)payload));  // convert to int
      break;
  }
}

void setup() {
  Serial.begin(115200); 

  // setup for the motors
  for (int i = 0; i < 3; i++) {
    pinMode(dirPin[i], OUTPUT);
    ledcAttach(pwmPin[i], pwmFreq, pwmResolution);
    ledcWrite(pwmPin[i], 0);
  }

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password); //connecting the esp32 to the given wifi network
  while (WiFi.status() != WL_CONNECTED) { //simple loop to show the wifi connection progress
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println("ESP32 IP address: " + WiFi.localIP().toString());

  webSocket.begin(); //initializes connection
  webSocket.onEvent(webSocketEvent); //receives commands

  Serial.println("WebSocket server started on port 81"); 
  Serial.println("Send 1=Fwd, 2=Bwd, 3=Right, 4=Left, 5=CW, 6=CCW"); //basic code we are using to do in switch case
}

void loop() {
  webSocket.loop(); //web socket keeps listening for commands
}
