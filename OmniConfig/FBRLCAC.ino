// Motor ORANGE: A (L298N #1); let less rpm motor be A
int in1 = 8;
int in2 = 10;
int enA = 9;

// Motor YELLOW: B (L298N #1)
int in3 = 2;
int in4 = 4;
int enB = 3;

// Motor RED: C (L298N #2)
int in5 = 6;
int in6 = 7;
int enC = 5;

void setup() {
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT); pinMode(in6, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Enter the direction; 1- forward; 2- backward; 3- right; 4- left; 5- clockwise; 6- anticlockwise ");
  
    while(Serial.available()==0){
  }
     	 int direction = Serial.parseInt();
    Serial.println(direction);
  
  movement(direction);
}

void movement(int dir) {
  switch (dir){
    case 1: 
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, 255*(23/25));

        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0);

        digitalWrite(in5, HIGH);
        digitalWrite(in6, LOW);
        analogWrite(enC, 255);
		break;
    
    case 2: 
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
    	  digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, 255*(23/25));

        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enB, 0);

        digitalWrite(in5, LOW);
        digitalWrite(in6, HIGH);
        analogWrite(enC, 255);
		break;

    case 6:
    	digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, 255*(23/25));

        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0);

        digitalWrite(in5, HIGH);
        digitalWrite(in6, LOW);
        analogWrite(enC, 255);
}
//one with low rpm; send pwm* 230.0/250.0 into that motor. 
//see the speed and decide respective pwm. 
