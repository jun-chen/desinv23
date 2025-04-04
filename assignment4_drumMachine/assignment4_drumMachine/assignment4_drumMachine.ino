// LDR input pins
const int ldrPins[3] = {A0, A1, A2};

// Solenoid output pins: [IN1, IN2]
const int solenoidPins[3][2] = {
  {2, 3},
  {4, 5},
  {6, 7}
};

int ldrThresholds[3];           // Dynamic thresholds after ambient calibration

const int calibrationTime = 3000;
const int pulseDuration = 100;      // Time solenoid stays on (ms)
const int triggerOffset = 100;      // Difference below ambient to trigger

// For state tracking
bool solenoidActive[3] = {false, false, false};
unsigned long solenoidStartTime[3] = {0, 0, 0};
unsigned long lastTriggerTime[3] = {0, 0, 0};
const int debounceInterval = 150;  // Debounce time (ms)

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 3; i++) {
    pinMode(solenoidPins[i][0], OUTPUT);
    pinMode(solenoidPins[i][1], OUTPUT);
  }

  Serial.println("Calibrating...");
  calibrateLDRs();
  Serial.println("Calibration finished");
}

void loop() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 3; i++) {
    int ldrValue = analogRead(ldrPins[i]);

    // Debug: print LDR values
    // Serial.print("LDR "); Serial.print(i); Serial.print(": ");
    // Serial.print(ldrValue); Serial.print(" / Thr: ");
    // Serial.println(ldrThresholds[i]);

    // Check if the LDR is blocked and debounce time passed
    if (ldrValue < ldrThresholds[i] &&
        !solenoidActive[i] &&
        (currentTime - lastTriggerTime[i] > debounceInterval)) {
      
      activateSolenoid(i);
      solenoidStartTime[i] = currentTime;
      lastTriggerTime[i] = currentTime;
      solenoidActive[i] = true;
    }

    // Check if active solenoid needs to be turned off
    if (solenoidActive[i] && (currentTime - solenoidStartTime[i] >= pulseDuration)) {
      deactivateSolenoid(i);
      solenoidActive[i] = false;
    }
  }
}

// Calibrate ambient light and set thresholds
void calibrateLDRs() {
  const int samples = 100;
  unsigned long startTime = millis();
  int sums[3] = {0, 0, 0};

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 3; i++) {
      sums[i] += analogRead(ldrPins[i]);
    }
    delay(calibrationTime / samples); // Still okay here since it's startup
  }

  for (int i = 0; i < 3; i++) {
    int avg = sums[i] / samples;
    ldrThresholds[i] = avg - triggerOffset;
    Serial.print("LDR "); Serial.print(i);
    Serial.print(" avg: "); Serial.print(avg);
    Serial.print(" --> threshold: "); Serial.println(ldrThresholds[i]);
  }
}

// Turn solenoid ON
void activateSolenoid(int index) {
  digitalWrite(solenoidPins[index][0], HIGH);
  digitalWrite(solenoidPins[index][1], LOW);
  Serial.print("Solenoid "); Serial.print(index); Serial.println(" ON");
}

// Turn solenoid OFF
void deactivateSolenoid(int index) {
  digitalWrite(solenoidPins[index][0], LOW);
  digitalWrite(solenoidPins[index][1], LOW);
  Serial.print("Solenoid "); Serial.print(index); Serial.println(" OFF");
}