/* Own a Snoopy Component Tester

To test all components to make sure they work properly

Created May 7th, 2025
By Jun Chen, Sunyu Jung

The Circuit:
* Button, LDR, and Ultrasonic Sensor as input
* 8 LEDs, and serial readout of sensor readings as output
  
  -----------------------------------------------------------
  • Reports the state of every input to the Serial Monitor every 100 ms  
  • LED animation:  
        1. All eight LEDs ON for 1 s  
        2. Each LED lights up in sequence (200 ms per LED)  
        3. Pause 250 ms  
        4. Repeat  
  Hardware‑only dependency:  NewPing library for the HC‑SR04‑style ultrasonic sensor
*/

#include <NewPing.h>

/* ---------- Pin assignments ---------- */
const int LDR_PIN      = A0;
const int BUTTON_PIN   = 13;
const int TRIG_PIN     = 11;
const int ECHO_PIN     = 12;
const int LED_PINS[8]  = {10, 9, 8, 7, 6, 5, 4, 3};

const byte  NUM_LEDS        = 8;
const unsigned int MAX_CM   = 300;      // Max range for NewPing

/* ---------- Objects & timers ---------- */
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_CM);

unsigned long lastSerial   = 0;
const  unsigned long SERIAL_PERIOD = 100;

unsigned long lastAnim     = 0;
int  animStep              = -1;            // –1  ➜ “all‑on” phase

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);         // HIGH = not pressed

  for (byte i = 0; i < NUM_LEDS; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
}

void loop() {
  unsigned long now = millis();

  /* ---- Periodic serial report ---- */
  if (now - lastSerial >= SERIAL_PERIOD) {
    bool  buttonPressed = (digitalRead(BUTTON_PIN) == LOW);   // 1 = pressed
    int   ldrValue      = analogRead(LDR_PIN);                // 0‑1023
    unsigned int distance = sonar.ping_cm();                  // 0 cm if out of range

    Serial.print("Button: ");
    Serial.print(buttonPressed);
    Serial.print(" | LDR: ");
    Serial.print(ldrValue);
    Serial.print(" | Distance (cm): ");
    Serial.println(distance);

    lastSerial = now;
  }

  /* ----   LED animation ---- */
  if (animStep == -1) {                     // *** Phase A: all LEDs ON ***
    for (byte i = 0; i < NUM_LEDS; ++i) digitalWrite(LED_PINS[i], HIGH);

    if (now - lastAnim >= 1000) {           // After 1 s ➜ move to phase B
      for (byte i = 0; i < NUM_LEDS; ++i) digitalWrite(LED_PINS[i], LOW);
      animStep  = 0;
      lastAnim  = now;
    }
  }
  else {                                    // *** Phase B: one‑by‑one ***
    if (now - lastAnim >= 200) {            // Change LED every 200 ms
      // Turn all off first
      for (byte i = 0; i < NUM_LEDS; ++i) digitalWrite(LED_PINS[i], LOW);

      // Light the current LED
      digitalWrite(LED_PINS[animStep], HIGH);

      // Advance to next LED
      animStep = (animStep + 1) % NUM_LEDS;

      if (animStep == 0) {                  // Completed the chase loop
        /* --- small delay before restarting animation --- */
        delay(250);                         // 250 ms pause
        // Ensure LEDs are off during the pause
        for (byte i = 0; i < NUM_LEDS; ++i) digitalWrite(LED_PINS[i], LOW);
        animStep = -1;                      // Back to “all on” phase
        lastAnim = millis();                // Reset timer after delay
      } else {
        lastAnim = now;
      }
    }
  }
}
