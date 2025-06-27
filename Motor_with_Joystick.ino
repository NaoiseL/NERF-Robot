#define JOYSTICK_Y_PIN A1  // Joystick Y-axis
#define MOTOR_DIR_PIN 8    // Motor direction pin
#define MOTOR_PWM_PIN 10   // PWM pin for speed control

void setup() {
    Serial.begin(115200);  // Debugging

    // Set motor control pins
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
}

void loop() {
    int yValue = analogRead(JOYSTICK_Y_PIN);  // Read Y-axis value

    // Debugging: Print joystick values
    Serial.print("Y-axis: ");
    Serial.println(yValue);

    // Define threshold values for forward, backward, and stop
    int center = 512;  // Neutral position (joystick centered)
    int deadZone = 500; // Large dead zone (to avoid small movements)

    // Determine motor movement
    if (yValue > center + deadZone) {  // Move forward
        moveForward(map(yValue, center + deadZone, 1023, 100, 255));  // Scale speed
    } 
    else if (yValue < center - deadZone) {  // Move backward
        moveBackward(map(yValue, 0, center - deadZone, 255, 100));  // Scale speed
    } 
    else {  // Stop motor (inside deadzone)
        stopMotor();
    }

    delay(10);  // Small delay for stability
}

void moveForward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, 1);  // Forward direction
    digitalWrite(MOTOR_PWM_PIN, HIGH);  // Set speed
    Serial.println("Moving Forward");
}

void moveBackward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, 0);  // Backward direction
    digitalWrite(MOTOR_PWM_PIN, HIGH); // Set speed
    Serial.println("Moving Backward");
}

void stopMotor() {
    analogWrite(MOTOR_PWM_PIN, 0);  // Stop motor
    Serial.println("Motor Stopped");
}
