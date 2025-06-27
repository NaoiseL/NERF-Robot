#include <Servo.h>

#define POT_Y_PIN A1  // Y-axis potentiometer
#define SERVO_Y_PIN 10 // Servo controlled by Y-axis

Servo servoY;

void setup() {
    Serial.begin(9600);  // Debugging
    servoY.attach(SERVO_Y_PIN);
    servoY.write(90); // Initialize the servo at the center (90 degrees)
}

void loop() {
    int yValue = analogRead(POT_Y_PIN);  // Read Y-axis potentiometer

    // Debugging: Print Y-axis values
    Serial.print("Y: ");
    Serial.println(yValue);

    // Map the Y-axis value (0-1023) to servo angle (30-150 degrees)
    int servoYAngle = map(yValue, 0, 1023, 30, 150);

    // Move the Y-axis servo based on joystick input
    servoY.write(servoYAngle);

    delay(10);  // Small delay for stability
}
