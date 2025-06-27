// Pin definitions
const int enablePin = 8;  // Pin 8 is connected to E1 (Enable)
const int motorPin = 9;   // Pin 9 is connected to M1 (Motor Positive)
const int dirPin = 2;     // Pin 2 is connected to DIR input (direction control)

void setup() {
  // Set motor control pins as outputs
  pinMode(enablePin, OUTPUT);  // Enable the motor driver
  pinMode(motorPin, OUTPUT);   // Motor positive terminal
  pinMode(dirPin, OUTPUT);     // Direction control

  // Enable the motor driver (E1 pin set to HIGH)
  digitalWrite(enablePin, HIGH);
}

void loop() {
  // Set motor direction to clockwise
  digitalWrite(dirPin, HIGH); // HIGH for clockwise, LOW for counterclockwise
  
  // Run motor in clockwise direction for 2 seconds
  delay(20000);
  
  // Stop motor
  digitalWrite(enablePin, LOW);   // Set E1 to LOW to stop the motor
}
