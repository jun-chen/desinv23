/* BeatBox

Play drums with your fingertips!

Created April 4th, 2025
By Jun Chen, Sunyu Jung

The Circuit:
* 3 LDR as input
* 3 Solenoid as output through two L298N H-Bridge

*/

// --- Constants & Pins ---
const int ldrPins[3] = {A0, A1, A2};

const int solenoidPins[3][2] = {
  {2, 3},
  {4, 5},
  {6, 7}
};

int ldrThresholds[3];               // Calculated thresholds
int lastLdrValues[3] = {0, 0, 0};   // For edge detection
bool solenoidActive[3] = {false, false, false};
unsigned long solenoidStartTime[3] = {0, 0, 0};

const int pulseDuration = 100;      // Solenoid ON time
const unsigned long calibrationTime = 3000; // 3 second calibration

// --- Setup ---
void setup() {
  Serial.begin(9600);

  // Set solenoid pins
  for (int i = 0; i < 3; i++) {
    pinMode(solenoidPins[i][0], OUTPUT);
    pinMode(solenoidPins[i][1], OUTPUT);
  }

  Serial.println("Running simple min-max LDR calibration...");
  calibrateLDRsMinMax();
  Serial.println("Calibration complete.");
}

// --- Main Loop ---
void loop() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 3; i++) {
    int currentLdr = analogRead(ldrPins[i]);

    // Detect transition from light to dark
    if (currentLdr < ldrThresholds[i] &&
        lastLdrValues[i] >= ldrThresholds[i] &&
        !solenoidActive[i]) {

      activateSolenoid(i);
      solenoidActive[i] = true;
      solenoidStartTime[i] = currentTime;
    }

    // Store current value for next loop
    lastLdrValues[i] = currentLdr;

    // Turn off solenoid after pulse duration
    if (solenoidActive[i] && (currentTime - solenoidStartTime[i] >= pulseDuration)) {
      deactivateSolenoid(i);
      solenoidActive[i] = false;
    }
  }
}

// --- Functions ---

void calibrateLDRsMinMax() {
  int ldrMin[3] = {1023, 1023, 1023}; // Highest possible at start
  int ldrMax[3] = {0, 0, 0};          // Lowest possible at start

  unsigned long startTime = millis();

  Serial.println("Wave hand over each LDR now...");

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 3; i++) {
      int value = analogRead(ldrPins[i]);
      if (value < ldrMin[i]) ldrMin[i] = value;
      if (value > ldrMax[i]) ldrMax[i] = value;
    }
    delay(5); // Small delay to avoid overwhelming the ADC
  }

  for (int i = 0; i < 3; i++) {
    ldrThresholds[i] = (ldrMin[i] + ldrMax[i]) / 2;

    Serial.print("LDR "); Serial.print(i);
    Serial.print(" min: "); Serial.print(ldrMin[i]);
    Serial.print(" max: "); Serial.print(ldrMax[i]);
    Serial.print(" => threshold: "); Serial.println(ldrThresholds[i]);
  }
}

void activateSolenoid(int index) {
  digitalWrite(solenoidPins[index][0], HIGH);
  digitalWrite(solenoidPins[index][1], LOW);
  //Serial.print("Solenoid "); Serial.print(index); Serial.println(" ON");
}

void deactivateSolenoid(int index) {
  digitalWrite(solenoidPins[index][0], LOW);
  digitalWrite(solenoidPins[index][1], LOW);
  //Serial.print("Solenoid "); Serial.print(index); Serial.println(" OFF");
}