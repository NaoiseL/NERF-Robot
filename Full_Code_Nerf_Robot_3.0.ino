#include <Wire.h>
#include <Servo.h>

// FXOS8700 Accelerometer & Magnetometer (I2C)
#define FXOS8700_ADDR 0x1E
#define FXOS8700_REGISTER_CTRL_REG1  0x2A
#define FXOS8700_REGISTER_OUT_X_MSB  0x01
#define FXOS8700_REGISTER_OUT_Z_MSB  0x05  

// Servo control pins (Aiming System)
#define SERVO_X_PIN 9  // Servo for panning (Yaw) → Connects to Servo Signal Pin
#define SERVO_Y_PIN 10 // Servo for tilting (Pitch) → Connects to Servo Signal Pin

// Car Movement (H-Bridge 1)
#define JOYSTICK_Y_PIN A1  // Joystick Y-axis → Connects to joystick Y-axis output
#define MOTOR_DIR_PIN 8    // Motor direction control → Connects to H-Bridge DIR Input
#define MOTOR_PWM_PIN 6    // Motor speed control (PWM) → Connects to H-Bridge PWM Input

// Firing Mechanism (Servo Trigger & Button)
#define BUTTON_PIN 2        // Fire button input → Connects to one side of the push button (other side goes to GND)
#define FIRE_SERVO_PIN 5    // Servo for firing trigger → Connects to Servo Signal Pin
#define CONTROL_PIN 4       // Controls resistance-lowering mechanism → Connects to transistor base/input

// Firing Motor (H-Bridge 2 - Constant Spinning)
#define FIRE_MOTOR_DIR 11   // Direction control → Connects to H-Bridge 2 DIR Input
#define FIRE_MOTOR_PWM 3    // Speed control (PWM) → Connects to H-Bridge 2 PWM Input

// Servo objects
Servo servoX, servoY, fireServo;

// Timing variables
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 50;  // Update every 50ms

// Sensor reading range
const int16_t SENSOR_MIN = -12000;
const int16_t SENSOR_MAX = 12000;

// Servo angle range
const int SERVO_MIN = 0;
const int SERVO_MAX = 150;

int initialX = 0;
int initialY = 0;

// Scaling factor to reduce servo movement to half
const float SCALING_FACTOR = 0.5;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize FXOS8700 Sensor (I2C)
    initializeSensor();

    // Attach servos
    servoX.attach(SERVO_X_PIN);   // Connects to servo control wire
    servoY.attach(SERVO_Y_PIN);   // Connects to servo control wire
    fireServo.attach(FIRE_SERVO_PIN); // Connects to firing servo signal pin

    // Set servos to neutral position
    servoX.write(90);
    servoY.write(90);
    fireServo.write(0);

    // Set motor control pins (H-Bridge 1)
    pinMode(MOTOR_DIR_PIN, OUTPUT);  // Controls direction of drive motor
    pinMode(MOTOR_PWM_PIN, OUTPUT);  // Controls speed of drive motor

    // Set firing mechanism
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Internal pull-up resistor for button
    pinMode(CONTROL_PIN, OUTPUT);       // Controls resistance-lowering mechanism
    digitalWrite(CONTROL_PIN, LOW);     // Default to HIGH resistance

    // Set firing motor control (H-Bridge 2)
    pinMode(FIRE_MOTOR_DIR, OUTPUT);  // Controls direction of firing motor
    pinMode(FIRE_MOTOR_PWM, OUTPUT);  // Controls speed of firing motor

    // Start firing motor spinning
    controlFiringMotor();
}

void loop() {
    if (millis() - lastUpdateTime >= updateInterval) {
        controlAiming();
        lastUpdateTime = millis();
    }
    controlMovement();
    controlFiring();
    delay(10);
}

// **Aiming Control (Accelerometer-based)**
void controlAiming() {
    int16_t x = readAxis(FXOS8700_REGISTER_OUT_X_MSB);
    int16_t z = readAxis(FXOS8700_REGISTER_OUT_X_MSB + 2);

    // Map raw sensor values to servo movement
    int servoXAngle = map(z, SENSOR_MIN, SENSOR_MAX, SERVO_MIN, SERVO_MAX);
    int servoYAngle = map(x, SENSOR_MIN, SENSOR_MAX, SERVO_MIN, SERVO_MAX);

    // Apply scaling factor to reduce movement to half
    servoXAngle = 60 + (servoXAngle - 90) * -(SCALING_FACTOR);
    servoYAngle = 90 + (servoYAngle - 90) * SCALING_FACTOR;

    servoX.write(servoXAngle);
    servoY.write(servoYAngle);
}

// **Car Movement Control (H-Bridge 1)**
void controlMovement() {
    int yValue = analogRead(JOYSTICK_Y_PIN); // Read joystick Y-axis
    int center = 512;   // Neutral joystick position
    int deadZone = 50;  // Prevents small fluctuations from moving the car

    if (yValue > center + deadZone) {
        moveForward(map(yValue, center + deadZone, 1023, 100, 255));
    } else if (yValue < center - deadZone) {
        moveBackward(map(yValue, 0, center - deadZone, 255, 100));
    } else {
        stopMotor();
    }
}

// Move forward (H-Bridge 1)
void moveForward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, HIGH);  // Set motor direction forward
    analogWrite(MOTOR_PWM_PIN, speed);  // Set speed using PWM
    Serial.println("Moving Forward");
}

// Move backward (H-Bridge 1)
void moveBackward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, LOW);  // Set motor direction backward
    analogWrite(MOTOR_PWM_PIN, speed); // Set speed using PWM
    Serial.println("Moving Backward");
}

// Stop motor
void stopMotor() {
    analogWrite(MOTOR_PWM_PIN, 0);  // Stop movement
    Serial.println("Motor Stopped");
}

// **Firing Mechanism Control (Servo Trigger)**
void controlFiring() {
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW) {  // If button is pressed  // Move servo to 90° to fire
        digitalWrite(CONTROL_PIN, LOW);  // Activate trigger
        Serial.println("Firing");
        delay(500);  // Return to default position
        digitalWrite(CONTROL_PIN, HIGH);
    }
}

// **Constant Spinning Firing Motor (H-Bridge 2)**
void controlFiringMotor() {
    digitalWrite(FIRE_MOTOR_DIR, HIGH);  // Set direction forward
    analogWrite(FIRE_MOTOR_PWM, 200);    // Set constant speed (0-255)
    Serial.println("Firing Motor Spinning...");
}

// **Read FXOS8700 Sensor Data**
int16_t readAxis(uint8_t msbReg) {
    int16_t axisValue = 0;

    Wire.beginTransmission(FXOS8700_ADDR);
    Wire.write(msbReg);
    Wire.endTransmission(false);
    Wire.requestFrom(FXOS8700_ADDR, 2);

    if (Wire.available() >= 2) {
        axisValue = (Wire.read() << 8) | Wire.read();
    }

    return axisValue;
}

// **Initialize FXOS8700 Sensor**
void initializeSensor() {
    Wire.beginTransmission(FXOS8700_ADDR);
    Wire.write(FXOS8700_REGISTER_CTRL_REG1);
    Wire.write(0x00);  // Standby mode
    Wire.endTransmission();

    Wire.beginTransmission(FXOS8700_ADDR);
    Wire.write(FXOS8700_REGISTER_CTRL_REG1);
    Wire.write(0x0F);  // Active mode
    Wire.endTransmission();

    Serial.println("FXOS8700 Sensor Initialized");
}