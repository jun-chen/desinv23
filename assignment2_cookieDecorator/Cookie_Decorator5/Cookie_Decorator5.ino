#include <Servo.h>
#include <math.h>
#include <L298N.h>   // Include the L298N library for the DC motor

//--------------------- Motor (Extruder) Definitions ---------------------//
// Define the pins for the L298N H-bridge controlling the DC motor.
// Adjust these pins as needed.
const int motorIn1Pin = 2;
const int motorIn2Pin = 3;
const int motorEnablePin = 4;
const int motorSpeed = 255;   // Speed value (0-255)

// Create an L298N motor object (assuming the library constructor takes (in1, in2, enable))
L298N dcMotor(motorIn1Pin, motorIn2Pin, motorEnablePin);

//--------------------- Platform and Arm Definitions ---------------------//
// Platform dimensions (cm)
const float platformWidth = 15.0;
const float platformHeight = 10.0;

// Servo mounting position in platform coordinates (top left corner)
const float servoBaseX = 0;
const float servoBaseY = platformHeight;  // 10 cm

// Arm segment lengths (in cm)
const float L1 = 10.0;
const float L2 = 10.0;

//--------------------- Data Structures ---------------------//
// Now each drawing point also includes an extrusion flag.
// Coordinates here are defined in platform coordinates.
struct Point {
  float x;
  float y;
  unsigned long duration; // Duration in milliseconds for the move
  bool extrude;         // If true, the frosting motor is on during the move.
};

//--------------------- Servo Definitions ---------------------//
// Create servo objects for shoulder and elbow
Servo servo1;  // Shoulder joint
Servo servo2;  // Elbow joint

// Current servo angles (in servo units, 0-180)
// Initial positions are defined relative to the servo’s perspective.
int currentAngle1 = 90; // starting angle for servo1 (pointing right)
int currentAngle2 = 90; // starting angle for servo2 (pointing right)

//--------------------- Inverse Kinematics ---------------------//
/*
  calculateServoAngles: Computes the servo angles using inverse kinematics.
  
  Parameters:
    x, y         - The target coordinate in the servo's coordinate system.
                   (after translation from platform coordinates)
    angle1, angle2 - References to store the computed angles (in degrees).
  
  Returns:
    true if the point is reachable, false otherwise.
  
  Adjustments:
    - Uses the "elbow up" configuration (theta2 = -acos(D)).
    - For servo1: computed 0° (pointing right) maps to 90°.
    - For servo2: relative elbow angle is mapped such that:
         theta2 = 0°   -> servo2 = 180° (forearm up)
         theta2 = -90° -> servo2 = 90°  (forearm to the right)
         theta2 = -180°-> servo2 = 0°   (forearm down)
*/
bool calculateServoAngles(float x, float y, float &angle1, float &angle2) {
  float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (abs(D) > 1) {  // Target unreachable
    return false;
  }
  
  // "Elbow up" configuration:
  float theta2 = -acos(D);
  float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
  
  // Map computed angles to the servo's perspective:
  angle1 = theta1 * 180 / PI + 90;          // For servo1
  angle2 = 180 + (theta2 * 180 / PI);         // For servo2
  
  return true;
}

//--------------------- Smooth Servo Motion ---------------------//
/*
  moveToCoordinate: Moves the arm to a given platform-based (x, y) coordinate 
  over a set duration.
  
  Parameters:
    platformX, platformY - The target coordinate in platform coordinates.
    duration             - The movement duration in milliseconds.
  
  This function translates the platform coordinate to the servo's coordinate system
  by subtracting the servo base position.
*/
void moveToCoordinate(float platformX, float platformY, unsigned long duration) {
  // Translate platform coordinates to servo coordinates.
  float adjustedX = platformX - servoBaseX;         // (servoBaseX is 0)
  float adjustedY = platformY - servoBaseY;         // For servoBaseY = 10, adjustedY = platformY - 10.
  
  float targetAngle1, targetAngle2;
  
  if (!calculateServoAngles(adjustedX, adjustedY, targetAngle1, targetAngle2)) {
    Serial.print("Target unreachable (Platform Coordinates): (");
    Serial.print(platformX);
    Serial.print(", ");
    Serial.print(platformY);
    Serial.println(")");
    return;
  }
  
  // Debug output: show platform coordinate, adjusted servo coordinate, and calculated angles.
  Serial.print("Moving to platform coordinate (");
  Serial.print(platformX);
  Serial.print(", ");
  Serial.print(platformY);
  Serial.print(") -> Servo coordinate (");
  Serial.print(adjustedX);
  Serial.print(", ");
  Serial.print(adjustedY);
  Serial.print(") with angles -> Shoulder: ");
  Serial.print(targetAngle1);
  Serial.print("°, Elbow: ");
  Serial.println(targetAngle2);
  
  // Convert target angles to integer values for servo.write()
  int tAngle1 = int(targetAngle1);
  int tAngle2 = int(targetAngle2);
  
  // Define interpolation steps
  int steps = 50;
  unsigned long stepDelay = duration / steps;
  
  // Interpolate and update servos gradually
  for (int i = 1; i <= steps; i++) {
    int newAngle1 = currentAngle1 + (int)(((tAngle1 - currentAngle1) / (float)steps) * i);
    int newAngle2 = currentAngle2 + (int)(((tAngle2 - currentAngle2) / (float)steps) * i);
    
    servo1.write(newAngle1);
    servo2.write(newAngle2);
    
    delay(stepDelay);
  }
  
  // Update current servo angles
  currentAngle1 = tAngle1;
  currentAngle2 = tAngle2;
}

//--------------------- Shape Drawing Function ---------------------//
/*
  drawShape: Moves the arm through a series of points defined in platform coordinates.
  
  For each point, the DC motor (extruder) is toggled on or off based on the 'extrude' flag.
  
  Parameters:
    shape     - An array of Point structures defining the shape.
    numPoints - The number of points in the shape.
*/
void drawShape(Point shape[], int numPoints) {
  for (int i = 0; i < numPoints; i++) {
    // Toggle the frosting motor based on the extrude flag
    if (shape[i].extrude) {
      Serial.println("Extruder ON");
      dcMotor.setSpeed(motorSpeed);
      dcMotor.forward();
    } else {
      Serial.println("Extruder OFF");
      dcMotor.stop();
    }
    
    Serial.print("Next platform coordinate: ");
    Serial.print(shape[i].x);
    Serial.print(", ");
    Serial.print(shape[i].y);
    Serial.print(" | Duration: ");
    Serial.print(shape[i].duration);
    Serial.println(" ms");
    
    moveToCoordinate(shape[i].x, shape[i].y, shape[i].duration);
    delay(100); // Small pause between moves
  }
  
  // Make sure to turn off the motor at the end
  dcMotor.stop();
}

//--------------------- Example Shape: 5-Point Star ---------------------//
// Define a star shape using platform coordinates (15 cm x 10 cm).
// The extrude flag indicates when to turn on the frosting motor.
// In this example, we start by moving to the start without extruding,
// then draw the star outline with extrusion.
Point starShape[] = {
  {7.5, 10.0, 2000, false},  // Move to start (top center) without extruding
  {12.0, 5.0, 2000, true},   // Draw star: extrude frosting
  {7.5, 0.0, 2000, true},
  {3.0, 5.0, 2000, true},
  {7.5, 10.0, 2000, true}    // Return to start with extrusion
};
int numStarPoints = sizeof(starShape) / sizeof(starShape[0]);

// TODO: Add more shapes with different coordinate arrays and extrusion flags for other number keys.

void setup() {
  // Attach servos to Arduino pins (update pin numbers if required)
  servo1.attach(9);
  servo2.attach(10);
  
  // Set initial servo positions (using servo-based angles)
  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
  
  // Initialize the DC motor by stopping it
  dcMotor.stop();
  
  // Start Serial communication for keyboard input
  Serial.begin(9600);
  Serial.println("Platform-based coordinates active with extruder control.");
  Serial.println("Press '1' to draw a star shape.");
  // TODO: Add more options for additional shapes.
}

void loop() {
  // Check for keyboard input from the Serial Monitor.
  if (Serial.available() > 0) {
    char ch = Serial.read();
    switch (ch) {
      case '1':
        Serial.println("Drawing star shape...");
        drawShape(starShape, numStarPoints);
        break;
      // TODO: Add cases for other shapes (e.g., '2', '3', etc.)
      default:
        Serial.println("Shape not defined. Please press a valid number key.");
        break;
    }
  }
}
