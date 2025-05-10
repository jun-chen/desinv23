/* Own a Snoopy

Have you ever watched Snoopy dance across the screen and thought, 
“Man, I wish I could just give that little guy a head scratch”? 
well, WISH NO MORE! Introducing OWN A SNOOPY, your very own interactive 
Snoopy experience!!!


Created May 7th, 2025
By Jun Chen, Sunyu Jung

The Circuit:
* Button, LDR, and Ultrasonic Sensor as input
* 8 LEDs, and serial as output

The Purpose:
* Interpret data from the three sensors
* Send serial messages accordingly to p5.js for handling visuals
*/

#include <NewPing.h>

// === PIN ASSIGNMENTS ===
const uint8_t LDR_PIN     = A0;
const uint8_t BUTTON_PIN  = 13;
const uint8_t TRIG_PIN    = 11;
const uint8_t ECHO_PIN    = 12;
const uint8_t LED_PINS[8] = {10,9,8,7,6,5,4,3};

// === ULTRASONIC SETUP ===
const unsigned int MAX_DISTANCE = 200; // cm
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// === DURATIONS ===
const unsigned long DUR_EAT = 3000;  // ms

// === HYSTERESIS THRESHOLDS ===
const unsigned int HOUSE_ON_CM  = 5;   // go “on” when < 5 cm
const unsigned int HOUSE_OFF_CM = 10;  // go “off” when > 10 cm

// === LDR CALIBRATION ===
int minLight = 1023;
int maxLight = 0;
int thresholdLight = 0;

// === STATE & FLAGS ===
enum State { IDLE, PET, EAT, HOUSE };
State currentState = IDLE;

static bool houseDetected = false;
static bool eatActive     = false;
static unsigned long eatStart = 0;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (auto p : LED_PINS) pinMode(p, OUTPUT);

  // calibrate LDR
  for (int i = 0; i < 100; i++) {
    int v = analogRead(LDR_PIN);
    minLight = min(minLight, v);
    maxLight = max(maxLight, v);
    delay(5);
  }
  thresholdLight = (minLight + maxLight) / 2;
}

void loop() {
  unsigned long now = millis();

  // --- 1) Finish EAT if active ---
  if (eatActive) {
    if (now - eatStart >= DUR_EAT) {
      eatActive = false;
      currentState = IDLE;
      Serial.println("IDLE");
    }
    // while eating, do nothing else
    delay(20);
    return;
  }

  // --- 2) Read sensors ---
  unsigned int dist = sonar.ping_cm();
  bool houseOn  = (dist > 0 && dist < HOUSE_ON_CM);
  bool houseOff = (dist > HOUSE_OFF_CM);

  int light = analogRead(LDR_PIN);
  minLight = min(minLight, light);
  maxLight = max(maxLight, light);
  thresholdLight = (minLight + maxLight) / 2;
  bool petTrig = (light < thresholdLight - 50);

  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  // --- 3) HOUSE with hysteresis ---
  if (houseOn && !houseDetected) {
    houseDetected = true;
    currentState = HOUSE;
    Serial.println("HOUSE");
  } 
  else if (houseDetected && houseOff) {
    // only clear when distance is comfortably above
    houseDetected = false;
    currentState = IDLE;
    Serial.println("IDLE");
  }

  // --- 4) EAT on single button press ---
  else if (buttonPressed && currentState != EAT) {
    eatActive   = true;
    eatStart    = now;
    currentState = EAT;
    Serial.println("EAT");
  }

  // --- 5) PET edge detection ---
  else if (petTrig && currentState != PET) {
    currentState = PET;
    Serial.println("PET");
  } 
  else if (!petTrig && currentState == PET) {
    currentState = IDLE;
    Serial.println("IDLE");
  }

  // --- 6) LED behavior ---
  if (currentState == HOUSE) {
    pulsateLEDs(now);
  } else {
    allLEDsOff();
  }

  delay(20); // small debounce
}

void allLEDsOff() {
  for (auto p : LED_PINS) digitalWrite(p, LOW);
}

void pulsateLEDs(unsigned long t) {
  for (int i = 0; i < 8; i++) {
    float phase = TWO_PI * i / 8.0;
    float val   = (sin((t / 200.0) + phase) + 1) * 127.5;
    analogWrite(LED_PINS[i], (int)val);
  }
}
