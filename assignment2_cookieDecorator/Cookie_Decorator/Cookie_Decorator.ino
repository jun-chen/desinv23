#include <Servo.h>

#include <L298N.h>

// Servo Motor Data Pins
const unsigned int servo1Pin = 6;
const unsigned int servo2Pin = 5;

// H-Bridge Data Pins
const unsigned int enableAPin = 10;
const unsigned int hInput1 = 9;
const unsigned int hInput2 = 8;

// Making Motor Objects
Servo servo1;
Servo servo2;
L298N frostingMotor(hInput1, hInput2, enableAPin);

// For keeping Time
unsigned long previousTime = 0;



// Servo Motors Data
int servo1Position = 90;
int servo2Position = 90;


void setup() {
  Serial.begin(9600);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(92);
  servo2.write(90);
}

void loop() {
  unsigned long currentTime = millis();
  
}
