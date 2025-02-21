/* 
The End Portal

When an Eye of Ender is placed on the last empty Portal Frame block, four LEDs light up to illuminate the End Portal

The Circuit:
* 1 LDR as input
* 4 Blue LED as output

Created February 10th, 2025
By Jun Chen
*/

// LED and LDR pins
const int led1 = 9;
const int led2 = 10;
const int led3 = 11;
const int led4 = 12;
const int LDR = A0;

// For time related use
unsigned long currentTime;
unsigned long lastTime;
unsigned long interval;

// For automatically adjusting to the current room's brightness
int sensorMax;
int sensorMin;

void setup() {
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(LDR, INPUT);

  // Base values for LDR readings
  sensorMax = 50;
  sensorMin = 60;
  
  // Delay between serial debug messages
  interval = 250;
}

void loop() {

  int lightValue = analogRead(LDR);

  // Automatically calibrate the LDR to the room's brightness
  if (lightValue > sensorMax) {
    sensorMax = lightValue;
  }

  // If LDR is reading below 50% of min and max, turn on LEDs
  if (lightValue < (sensorMax - (sensorMax - sensorMin) / 2)) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
  } else {
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
  }

  // Print serial messages every 250ms without using delay() and pausing the entire program
  currentTime = millis();

  if ((currentTime - lastTime) >= interval) {
    lastTime = currentTime;
    String msg = "Min=" + String(sensorMin) + " Max=" + String(sensorMax) + " Current=" + String(lightValue);
    Serial.println(msg);
  }

}
