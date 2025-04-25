#include <NewPing.h>

// === PIN ASSIGNMENTS ===
const uint8_t LDR_PIN      = A0;
const uint8_t BUTTON_PIN   = 13;
const uint8_t TRIG_PIN     = 11;
const uint8_t ECHO_PIN     = 12;
const uint8_t LED_PINS[8]  = {10,9,8,7,6,5,4,3};  // G1,G2,Y1,Y2,R1,R2,B1,B2

// === ULTRASONIC SETUP ===
const unsigned int MAX_DISTANCE = 200; // cm
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// === STATE MACHINE ===
enum State { IDLE, PET, EAT, HOUSE };
State currentState = IDLE;
unsigned long stateStart   = 0;
unsigned long stateDur     = 0;

// durations (ms)
const unsigned long DUR_PET   = 2000;
const unsigned long DUR_EAT   = 5000;
const unsigned long DUR_HOUSE = 5000;

// === LDR CALIBRATION ===
int minLight = 1023;
int maxLight = 0;
int thresholdLight = 0;

void setup() {
  Serial.begin(115200);
  
  // inputs
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // leds
  for (auto p : LED_PINS) pinMode(p, OUTPUT);
  
  // initial LDR calibration (read 100 samples)
  for (int i = 0; i < 100; i++) {
    int v = analogRead(LDR_PIN);
    minLight = min(minLight, v);
    maxLight = max(maxLight, v);
    delay(10);
  }
  thresholdLight = (minLight + maxLight) / 2;
}

void loop() {
  unsigned long now = millis();
  
  // If in an animation state, check for finish
  if (currentState != IDLE) {
    if (now - stateStart >= stateDur) {
      goIdle();
    } else {
      // During HOUSE state, pulsate LEDs
      if (currentState == HOUSE) pulsateLEDs(now);
      return;  // wait for animation to finish
    }
  }
  
  // === IDLE: check new triggers in priority order ===
  // 1) House (ultrasonic sees object very close)
  unsigned int dist = sonar.ping_cm();
  if (dist > 0 && dist < 5) {
    enterState(HOUSE, DUR_HOUSE);
    return;
  }
  
  // 2) Eating (button press)
  if (digitalRead(BUTTON_PIN) == LOW) {
    enterState(EAT, DUR_EAT);
    return;
  }
  
  // 3) Petting (sudden dip in brightness)
  int light = analogRead(LDR_PIN);
  // dynamic adjust min/max ambient
  minLight = min(minLight, light);
  maxLight = max(maxLight, light);
  thresholdLight = (minLight + maxLight) / 2;
  
  if (light < thresholdLight - 50) {  // adjust hysteresis as needed
    enterState(PET, DUR_PET);
    return;
  }
  
  // still idle
}

void enterState(State s, unsigned long dur) {
  currentState = s;
  stateStart  = millis();
  stateDur    = dur;
  
  // send command over Serial
  switch (s) {
    case PET:   Serial.println("PET");   break;
    case EAT:   Serial.println("EAT");   break;
    case HOUSE: Serial.println("HOUSE"); break;
    default:    Serial.println("IDLE");  break;
  }
  
  // if entering HOUSE, start LEDs; else turn them off
  if (s == HOUSE) {
    // nothing here—the pulsateLEDs() in loop() handles it
  } else {
    allLEDsOff();
  }
}

void goIdle() {
  currentState = IDLE;
  Serial.println("IDLE");
  allLEDsOff();
}

void allLEDsOff() {
  for (auto p : LED_PINS) digitalWrite(p, LOW);
}

// simple sine‐wave pulsation on 8 LEDs with phase offsets
void pulsateLEDs(unsigned long t) {
  for (int i = 0; i < 8; i++) {
    // map sine value to 0–255 PWM
    float phase = TWO_PI * i / 8.0;
    float val   = (sin((t / 200.0) + phase) + 1) / 2.0;  
    analogWrite(LED_PINS[i], (int)(val * 255));
  }
}
