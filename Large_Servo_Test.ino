#include <Servo.h>

Servo myServo;  // Create a servo object
const int servoPin = 9;  // Pin connected to the servo signal

void setup() {
    myServo.attach(servoPin); // Attach the servo to the pin
    Serial.begin(9600);
    Serial.println("Servo Test Initialized");
}

void loop() {
    // Sweep from 0 to 180 degrees
    for (int pos = 0; pos <= 180; pos += 5) {
        myServo.write(pos);
        Serial.print("Position: ");
        Serial.println(pos);
        delay(300);
    }
    
    // Sweep from 180 to 0 degrees
    for (int pos = 180; pos >= 0; pos -= 5) {
        myServo.write(pos);
        Serial.print("Position: ");
        Serial.println(pos);
        delay(300);
    }
}