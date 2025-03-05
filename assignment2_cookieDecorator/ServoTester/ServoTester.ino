#include <Servo.h>

Servo myServo;
int calibrationOffset = 0; // Calibration offset added to the commanded angle
int currentAngle = 0;      // Last commanded angle (after calibration adjustment)

void setup() {
  Serial.begin(9600);       // Start serial communication
  myServo.attach(9);        // Attach the servo to pin 9
  myServo.write(currentAngle);  // Set initial servo position

  // Print instructions for use
  Serial.println("Commands:");
  Serial.println("For setting servo angle, type: A<angle> (e.g., A90)");
  Serial.println("For setting calibration offset, type: C<offset> (e.g., C5 or C-5)");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read full line until newline
    input.trim(); // Remove extra whitespace

    if (input.length() > 0) {
      char command = input.charAt(0);  // Get the command type (A or C)
      String valueStr = input.substring(1); // The rest is the value
      valueStr.trim(); 
      int value = valueStr.toInt();    // Convert to integer

      if (command == 'A' || command == 'a') {
        // Apply calibration offset to the commanded angle
        int targetAngle = value + calibrationOffset;
        // Clamp the result between 0 and 180 degrees
        if (targetAngle < 0) targetAngle = 0;
        if (targetAngle > 180) targetAngle = 180;
        currentAngle = targetAngle;
        myServo.write(currentAngle);
        Serial.print("Servo angle set to (after calibration): ");
        Serial.println(currentAngle);
      } 
      else if (command == 'C' || command == 'c') {
        calibrationOffset = value;
        Serial.print("Calibration offset set to: ");
        Serial.println(calibrationOffset);
      }
      else {
        Serial.println("Invalid command. Use A<angle> or C<offset>.");
      }
    }
  }
}
