#include <Servo.h>

// Define the PWM pin for the motor control signal
const int motorPin = 3;  // Change as needed

Servo motorServo;

void setup() {
  Serial.begin(9600);
  motorServo.attach(motorPin);
  
  // Set initial (neutral) position: 1500 μsec pulse width
  motorServo.writeMicroseconds(1500);
  Serial.println("Motor ready.");
  Serial.println("Enter an angle between -135 and +135 (0 = neutral):");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the input string until newline
    String input = Serial.readStringUntil('\n');
    float angle = input.toInt();  // Expected input: angle in degrees
    
    // Constrain angle to the valid range (-135° to +135°)
    
    
    // Map the angle to the corresponding pulse width:
    // -135°  -> 500 μs, 0° -> 1500 μs, +135° -> 2500 μs
    int pulseWidth = ceil (((2000.0/270)*angle)+500);
    
    
    
    // Write the calculated pulse width to the motor
    motorServo.writeMicroseconds(pulseWidth);
    
    // Feedback to Serial Monitor
    Serial.print("Angle set to: ");
    Serial.print(angle);
    Serial.print("°, Pulse Width: ");
    Serial.print(pulseWidth);
    Serial.println(" μs");
  }
}
