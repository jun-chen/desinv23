#include <Servo.h>
#include <math.h>

// Define a structure for a 2D point that includes a duration for movement.
struct Point {
  float x;
  float y;
  unsigned long duration;  // Duration in milliseconds to move to this point.
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

// Starting angles for the servos (in degrees).
// For servo2 these are unadjusted target angles.
float currentAngle1 = 90.0;
float currentAngle2 = 90.0;

// Button pin definitions.
const int buttonShape1 = 2;  // Button to trigger shape 1
// TODO: Define additional button pins for other shapes if needed

// Placeholder array for shape 1 coordinates and individual durations.
// TODO: Fill in or adjust the coordinates and durations as needed.
Point shape1[] = {
  {10, 0, 1000},   // Move to (10, 0) in 1000ms
  {7, 7, 1500},    // Move to (7, 7) in 1500ms
  {0, 10, 1200}    // Move to (0, 10) in 1200ms
};
int shape1Length = sizeof(shape1) / sizeof(shape1[0]);

// Computes inverse kinematics for a 2-link arm.
// Given target (x, y), computes the servo angles for the shoulder and elbow.
bool computeIK(float x, float y, float &angle1, float &angle2) {
  float d = sqrt(x * x + y * y);
  
  // Check if the point is reachable.
  if (d > (L1 + L2)) {
    Serial.println("Target point unreachable");
    return false;
  }
  
  float cosAngle2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  cosAngle2 = constrain(cosAngle2, -1.0, 1.0);
  angle2 = acos(cosAngle2);  // Elbow down configuration
  
  float k1 = L1 + L2 * cos(angle2);
  float k2 = L2 * sin(angle2);
  angle1 = atan2(y, x) - atan2(k2, k1);
  
  // Convert radians to degrees.
  angle1 = angle1 * 180.0 / PI;
  angle2 = angle2 * 180.0 / PI;
  
  return true;
}

// Moves the arm to the (x, y) coordinate over the specified duration.
void moveArmTo(float x, float y, unsigned long duration) {
  float targetAngle1, targetAngle2;
  if (!computeIK(x, y, targetAngle1, targetAngle2)) {
    return;
  }
  
  Serial.print("Moving to (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") over ");
  Serial.print(duration);
  Serial.println(" ms");
  
  Serial.print("Target servo angles (unadjusted): Shoulder = ");
  Serial.print(targetAngle1);
  Serial.print("°, Elbow = ");
  Serial.print(targetAngle2);
  Serial.println("°");
  
  // Calculate adjusted servo2 target: apply the 90° clockwise offset.
  float targetServo2Command = targetAngle2 - 90;
  Serial.print("Adjusted target for Servo2 = ");
  Serial.print(targetServo2Command);
  Serial.println("°");
  
  unsigned long stepDelay = 20;  // Delay between interpolation steps (ms)
  int steps = duration / stepDelay;
  
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float newAngle1 = currentAngle1 + t * (targetAngle1 - currentAngle1);
    float newAngle2 = currentAngle2 + t * (targetAngle2 - currentAngle2);
    
    servo1.write(newAngle1);
    // For servo2, apply the 90° offset (subtract 90°)
    servo2.write(newAngle2 - 90);
    
    // Print intermediate step information.
    Serial.print("Step ");
    Serial.print(i);
    Serial.print(": Shoulder = ");
    Serial.print(newAngle1);
    Serial.print("°, Elbow (unadjusted) = ");
    Serial.print(newAngle2);
    Serial.print("°, Servo2 command = ");
    Serial.print(newAngle2 - 90);
    Serial.println("°");
    
    delay(stepDelay);
  }
  
  // Update current angles after moving.
  currentAngle1 = targetAngle1;
  currentAngle2 = targetAngle2;
  Serial.print("Reached: Shoulder = ");
  Serial.print(currentAngle1);
  Serial.print("°, Elbow (unadjusted) = ");
  Serial.print(currentAngle2);
  Serial.print("°, Servo2 command = ");
  Serial.print(currentAngle2 - 90);
  Serial.println("°");
}

// Draws a shape by moving to each point using its individual duration.
void drawShape(Point shape[], int length) {
  for (int i = 0; i < length; i++) {
    moveArmTo(shape[i].x, shape[i].y, shape[i].duration);
    delay(200);  // Delay between moves; adjust as needed.
  }
}

void setup() {
  // Attach servos.
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  
  // Move servos to their starting positions.
  servo1.write(currentAngle1);
  // For servo2, apply the offset when writing the initial position.
  servo2.write(currentAngle2 - 90);
  
  // Configure button pin with internal pull-up resistor.
  pinMode(buttonShape1, INPUT_PULLUP);
  // TODO: Set up additional buttons with pinMode() if needed.
  
  Serial.begin(9600);
}

void loop() {
  // Check if the button for Shape 1 is pressed (active LOW).
  if (digitalRead(buttonShape1) == LOW) {
    Serial.println("Drawing Shape 1");
    drawShape(shape1, shape1Length);
    delay(500);  // Debounce delay.
  }
  
  // TODO: Add additional conditions for other shape buttons.
}
