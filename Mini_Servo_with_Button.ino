#include <Servo.h>

#define BUTTON_PIN 2
#define SERVO_PIN 9
#define CONTROL_PIN 4  // Controls the resistance-lowering mechanism

Servo myServo;
bool buttonState = false;

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Internal pull-up resistor
    pinMode(CONTROL_PIN, OUTPUT);       // Output to control resistance
    myServo.attach(SERVO_PIN);
    
    myServo.write(0); // Start position of servo
    digitalWrite(CONTROL_PIN, LOW); // Default state (high resistance)
}

void loop() {
    buttonState = digitalRead(BUTTON_PIN); 

    if (buttonState == LOW) {  // Button pressed
        myServo.write(90);  // Move servo to 90 degrees
        digitalWrite(CONTROL_PIN, HIGH);  // Lower resistance (e.g., enable transistor)
    } else {  
        myServo.write(0);  // Move servo back
        digitalWrite(CONTROL_PIN, LOW);  // Keep high resistance
    }
}
