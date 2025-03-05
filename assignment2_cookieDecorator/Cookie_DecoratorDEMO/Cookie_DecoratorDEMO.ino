#include <Servo.h>
#include <math.h>

// Structure to store a 2D point and the duration to move to that point.
struct Point {
  float x;              // x in range 0-40
  float y;              // y in range 0-40
  unsigned long duration;  // Duration in milliseconds.
};

// Servo objects for the shoulder and elbow.
Servo servo1;
Servo servo2;

// Servo pins.
const int servo1Pin = 9;
const int servo2Pin = 10;

// Arm segment lengths (adjust these to match your build).
float L1 = 10.0;
float L2 = 10.0;

// Offsets for servo mounting.
const float servo1Offset = 90.0;  // Added to computed angle for servo1.
// Servo2 still uses a -90° offset.
  
// Computed angles (in the “internal” space centered at (0,0)) are stored without offsets.
float currentAngle1 = 0.0;   // For servo1 (computed value)
float currentAngle2 = 90.0;  // For servo2 (computed value)

// Shape1 now defines a 5-point star in the 0–40 coordinate system.
// These points are the result of taking the original star (centered at (0,0)) and adding 20 to both x and y.
Point shape1[] = {
  {20.00, 30.00, 1000},  // V0: Top
  {25.88, 11.91, 1000},  // V2: Bottom Right
  {10.49, 23.09, 1000},  // V4: Left
  {29.51, 23.09, 1000},  // V1: Right
  {14.12, 11.91, 1000}   // V3: Bottom Left
};
int shape1Length = sizeof(shape1) / sizeof(shape1[0]);

// Computes inverse kinematics for a 2-link arm.
// The (x,y) passed in is in internal coordinates (centered at 0,0).
// Returns the computed servo angles for the shoulder (angle1) and elbow (angle2) in degrees.
bool computeIK(float x, float y, float &angle1, float &angle2) {
  float d = sqrt(x * x + y * y);
  if (d > (L1 + L2)) {
    return false;  // Target unreachable.
  }
  
  float cosAngle2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  cosAngle2 = constrain(cosAngle2, -1.0, 1.0);
  angle2 = acos(cosAngle2);  // Elbow down configuration.
  
  float k1 = L1 + L2 * cos(angle2);
  float k2 = L2 * sin(angle2);
  angle1 = atan2(y, x) - atan2(k2, k1);
  
  // Convert from radians to degrees.
  angle1 = angle1 * 180.0 / PI;
  angle2 = angle2 * 180.0 / PI;
  
  return true;
}

// Moves the arm to the (x, y) coordinate (given in 0–40 space) over the specified duration.
void moveArmTo(float x, float y, unsigned long duration) {
  // Convert input coordinates from 0-40 space to internal coordinates (centered at 0,0):
  float internalX = x - 20;
  float internalY = y - 20;
  
  float targetAngle1, targetAngle2;
  if (!computeIK(internalX, internalY, targetAngle1, targetAngle2)) {
    Serial.println("Target unreachable.");
    return;
  }
  
  // Adjust target angles: add offset for servo1 and subtract 90 for servo2.
  float adjustedTarget1 = targetAngle1 + servo1Offset;
  float adjustedTarget2 = targetAngle2 - 90;
  
  Serial.print("Moving to (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y);
  Serial.print(") [internal: (");
  Serial.print(internalX); Serial.print(", ");
  Serial.print(internalY);
  Serial.print(")] with target angles: Servo1 = ");
  Serial.print(adjustedTarget1);
  Serial.print("°, Servo2 = ");
  Serial.print(adjustedTarget2);
  Serial.println("°");
  
  unsigned long stepDelay = 20;  // Delay between interpolation steps.
  int steps = duration / stepDelay;
  
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float newComputed1 = currentAngle1 + t * (targetAngle1 - currentAngle1);
    float newComputed2 = currentAngle2 + t * (targetAngle2 - currentAngle2);
    
    servo1.write(newComputed1 + servo1Offset);
    servo2.write(newComputed2 - 90);
    
    delay(stepDelay);
  }
  
  // Update current angles (in computed space, without offsets).
  currentAngle1 = targetAngle1;
  currentAngle2 = targetAngle2;
  
  Serial.print("Reached (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y);
  Serial.print("): Final angles: Servo1 = ");
  Serial.print(currentAngle1 + servo1Offset);
  Serial.print("°, Servo2 = ");
  Serial.print(currentAngle2 - 90);
  Serial.println("°");
}

// Draws a shape by moving to each point in the provided array.
void drawShape(Point shape[], int length) {
  for (int i = 0; i < length; i++) {
    Serial.print("Drawing point ");
    Serial.print(i + 1);
    Serial.print(" of ");
    Serial.print(length);
    Serial.print(": (");
    Serial.print(shape[i].x);
    Serial.print(", ");
    Serial.print(shape[i].y);
    Serial.print(") with duration ");
    Serial.print(shape[i].duration);
    Serial.println(" ms");
    
    moveArmTo(shape[i].x, shape[i].y, shape[i].duration);
    delay(200);  // Brief delay between moves.
  }
}

void setup() {
  // Attach servos.
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  
  // Initialize servos to the starting positions.
  // The starting position is defined in internal space: currentAngle1 and currentAngle2.
  // Here, we convert to servo commands:
  servo1.write(currentAngle1 + servo1Offset);
  servo2.write(currentAngle2 - 90);  // Apply offset for servo2.
  
  // Begin Serial communication.
  Serial.begin(9600);
  Serial.println("Keyboard demo for drawing robot (coordinate range: 0-40).");
  Serial.println("Type '1' and press Enter to draw the Star Shape.");
  Serial.print("Initial servo positions: Servo1 = ");
  Serial.print(currentAngle1 + servo1Offset);
  Serial.print("°, Servo2 = ");
  Serial.print(currentAngle2 - 90);
  Serial.println("°");
}

void loop() {
  // Wait for keyboard input from the Serial Monitor.
  if (Serial.available() > 0) {
    char inputChar = Serial.read();
    if (inputChar == '1') {
      Serial.println("Command received: Draw Star Shape");
      drawShape(shape1, shape1Length);
      Serial.println("Done drawing Star Shape. Type '1' to draw again.");
    } else {
      Serial.print("Unrecognized input: ");
      Serial.println(inputChar);
      Serial.println("Please type '1' to draw the Star Shape.");
    }
  }
}
