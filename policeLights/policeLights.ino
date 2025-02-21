/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. 
  On the UNO it is attached to digital pin 13

  This example code is modified from.
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

int red = 13;  // define a variable to hold the pin number of the internal LED
int blue = 12;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin led as an output.
  Serial.begin(9600);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);

}

// the loop function runs over and over again forever
void loop() {
  rapid();
  rapid();
  slow();

}

void rapid() {
  flashRed();
  flashRed();
  flashRed();
  flashRed();
  flashBlue();
  flashBlue();
  flashBlue();
  flashBlue();
}

void slow() {
  slowRed();
  slowBlue();
  slowRed();
  slowBlue();
  slowRed();
  slowBlue();
  slowRed();
  slowBlue();
}

void flashRed() {
  digitalWrite(red, HIGH);
  delay(50);
  digitalWrite(red, LOW);
  delay(50);
}

void flashBlue() {
  digitalWrite(blue, HIGH);
  delay(50);
  digitalWrite(blue, LOW);
  delay(50);
}
void slowRed() {
  digitalWrite(red, HIGH);
  delay(250);
  digitalWrite(red, LOW);
  delay(250);
}

void slowBlue() {
  digitalWrite(blue, HIGH);
  delay(250);
  digitalWrite(blue, LOW);
  delay(250);
}