#include <Servo.h>
#include <math.h>

// Define a struct to store a coordinate with a move duration.
// Note: Coordinates are now defined in platform coordinates.
struct Point {
  float x;
  float y;
  unsigned long duration; // Duration in milliseconds
};

// Create servo objects for shoulder and elbow
Servo servo1;  // Shoulder joint
Servo servo2;  // Elbow joint

// Arm segment lengths (in cm) - adjust as needed
const float L1 = 10.0;  // Length of first arm segment
const float L2 = 10.0;  // Length of second arm segment

// Define the servo's mounting (base) position in platform coordinates.
// For a platform 15 cm wide x 10 cm tall, with the servo at the top left:
const float servoBaseX = 0;
const float servoBaseY = 10;

// Current servo angles (0-180) from the servo's perspective
int currentAngle1 = 90; // starting angle for servo1 (pointing right)
int currentAngle2 = 90; // starting angle for servo2 (pointing right)

//--------------------- Inverse Kinematics ---------------------//
/*
  calculateServoAngles: Computes the servo angles using inverse kinematics.
  
  Parameters:
    x, y         - The target coordinate in the servo's coordinate system.
                   (i.e. after translating from platform coordinates)
    angle1, angle2 - References to store the computed angles (in degrees).
  
  Returns:
    true if the point is reachable, false otherwise.
  
  Adjustments:
    - Uses the "elbow up" configuration (theta2 = -acos(D)).
    - For servo1: 0° (pointing right) becomes 90°.
    - For servo2: Relative elbow angle is mapped so that:
         theta2 = 0°  -> servo2 = 180° (forearm up)
         theta2 = -90°-> servo2 = 90°  (forearm to the right)
         theta2 = -180°-> servo2 = 0°  (forearm down)
*/
bool calculateServoAngles(float x, float y, float &angle1, float &angle2) {
  float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (abs(D) > 1) {  // Target unreachable
    return false;
  }
  
  // Use the negative acos for "elbow up" configuration.
  float theta2 = -acos(D);
  float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
  
  // Convert to degrees and adjust for the servo's perspective.
  angle1 = theta1 * 180 / PI + 90;          // Maps computed 0° (pointing right) to 90° for servo1.
  angle2 = 180 + (theta2 * 180 / PI);         // Maps elbow joint: 0 -> 180, -90 -> 90, -180 -> 0 for servo2.
  
  return true;
}

//--------------------- Smooth Servo Motion ---------------------//
/*
  moveToCoordinate: Moves the arm to a given platform-based (x, y) coordinate 
  over a set duration.
  
  Parameters:
    x, y     - The target coordinate in platform coordinates.
    duration - The time (in milliseconds) over which the movement should occur.
  
  This function translates the platform coordinate to the servo's coordinate system
  by subtracting the servo base position.
*/
void moveToCoordinate(float platformX, float platformY, unsigned long duration) {
  // Translate platform coordinates to servo coordinates.
  // Servo base is at (servoBaseX, servoBaseY) in platform coordinates.
  float adjustedX = platformX - servoBaseX;     // In this case, adjustedX = platformX.
  float adjustedY = platformY - servoBaseY;     // For servoBaseY = 10, adjustedY = platformY - 10.
  
  float targetAngle1, targetAngle2;
  
  if (!calculateServoAngles(adjustedX, adjustedY, targetAngle1, targetAngle2)) {
    Serial.print("Target unreachable (Platform Coordinates): (");
    Serial.print(platformX);
    Serial.print(", ");
    Serial.print(platformY);
    Serial.println(")");
    return;
  }
  
  // Print platform coordinate, adjusted (servo) coordinate, and calculated angles.
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
  
  // Define the number of interpolation steps
  int steps = 50;
  unsigned long stepDelay = duration / steps;
  
  // Calculate the increment per step for each servo
  float stepIncrement1 = (tAngle1 - currentAngle1) / (float)steps;
  float stepIncrement2 = (tAngle2 - currentAngle2) / (float)steps;
  
  for (int i = 1; i <= steps; i++) {
    int newAngle1 = currentAngle1 + (int)(stepIncrement1 * i);
    int newAngle2 = currentAngle2 + (int)(stepIncrement2 * i);
    
    servo1.write(newAngle1);
    servo2.write(newAngle2);
    
    delay(stepDelay);
  }
  
  // Update current angles to the new target positions
  currentAngle1 = tAngle1;
  currentAngle2 = tAngle2;
}

//--------------------- Shape Drawing Function ---------------------//
/*
  drawShape: Moves the arm through a series of points defined in platform coordinates.
  
  Parameters:
    shape     - An array of Point structures defining the shape.
    numPoints - The number of points in the shape.
*/
void drawShape(Point shape[], int numPoints) {
  for (int i = 0; i < numPoints; i++) {
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
}

//--------------------- Example Shape: 5-Point Star ---------------------//
// Updated star shape defined in platform coordinates:
// The star will start at the top, then go to the bottom left, then to the middle right,
// then to the middle left, then to the bottom right, and finally back to the top.
Point starShape[] = {
  {7.5, 9.0, 5000},    // Top center (inside top border)
  {5.0, 2.0, 5000},    // Bottom left
  {11.0, 6.0, 5000},   // Middle right
  {4.0, 6.0, 5000},    // Middle left
  {10.0, 2.0, 5000},   // Bottom right
  {7.5, 9.0, 5000}     // Return to top center
};
int numStarPoints = sizeof(starShape) / sizeof(starShape[0]);

// TODO: Add more shapes with different coordinate arrays for other number keys.

void setup() {
  // Attach servos to Arduino pins (change pin numbers if required)
  servo1.attach(9);
  servo2.attach(10);
  
  // Set initial positions (using servo-based angles)
  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
  
  // Start serial communication for keyboard input
  Serial.begin(9600);
  Serial.println("Platform-based coordinates are active.");
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
