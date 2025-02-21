int sensorValueBase;
int sensorValue;
int red;
int green;
int blue;
int brightness;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  int sensorValueBase = 430;
  red = 9;
  green = 10;
  blue = 11;
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

void loop() {
  sensorValue = analogRead(A0);
  int sensorValueConstrained = constrain(sensorValue, 100, 500);
  brightness = map(sensorValueConstrained, 100, 500, 0, 50);
  analogWrite(blue, brightness);
  analogWrite(red, 0);
  analogWrite(green, 0);

  Serial.println(brightness);
}
