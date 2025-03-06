/*
Author  : Andrea Lombardo
Site    : https://www.lombardoandrea.com
Source  : https://github.com/AndreaLombardo/L298N/

Here you can see how to work in a common configuration. 

Speed range go from 0 to 255, default is 100.
Use setSpeed(speed) to change.

Sometimes at lower speed motors seems not running.
It's normal, may depends by motor and power supply.

Wiring schema in file "L298N - Schema_with_EN_pin.png"
*/

// Include the library
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 8;
const unsigned int IN2 = 7;
const unsigned int EN = 9;

// Create one motor instance
L298N motor(EN, IN1, IN2);

void setup()
{
  // Used to display information
  Serial.begin(9600);

  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    //do nothing
  }

  // Set initial speed
  motor.setSpeed(100);
}

void loop()
{
  motor.forward();
}

/*
Print some informations in Serial Monitor
*/
void printSomeInfo()
{
  Serial.print("Motor is moving = ");
  Serial.print(motor.isMoving());
  Serial.print(" at speed = ");
  Serial.println(motor.getSpeed());
}