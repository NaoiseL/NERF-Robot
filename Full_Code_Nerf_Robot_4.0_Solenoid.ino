#include <Wire.h>
#include <Servo.h>

// FXOS8700 Accelerometer & Magnetometer (I2C)
#define FXOS8700_ADDR 0x1E
#define FXOS8700_REGISTER_CTRL_REG1  0x2A
#define FXOS8700_REGISTER_OUT_X_MSB  0x01
#define FXOS8700_REGISTER_OUT_Z_MSB  0x05  

// Servo control pins (Aiming System)
#define SERVO_X_PIN 9  // Servo for panning (Yaw)
#define SERVO_Y_PIN 10 // Servo for tilting (Pitch)

// Car Movement (H-Bridge 1)
#define JOYSTICK_Y_PIN A1  // Joystick Y-axis
#define MOTOR_DIR_PIN 8    // Motor direction control
#define MOTOR_PWM_PIN 6    // Motor speed control (PWM)

// Firing Mechanism
#define BUTTON_PIN 2        // Fire button input
#define FIRE_SERVO_PIN 5    // Servo for firing trigger
#define CONTROL_PIN 4       // Controls resistance-lowering mechanism

// Firing Motor (H-Bridge 2)
#define FIRE_MOTOR_DIR 11   // Direction control
#define FIRE_MOTOR_PWM 3    // Speed control (PWM)

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

// Scaling factor to reduce servo movement
const float SCALING_FACTOR = 0.5;

// Smoothing variables
const float SMOOTHING_FACTOR = 0.2;
float filteredServoX = 90; // Start at neutral position
float filteredServoY = 90;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize FXOS8700 Sensor (I2C)
    initializeSensor();

    // Attach servos
    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    fireServo.attach(FIRE_SERVO_PIN);

    // Set servos to neutral position
    servoX.write(90);
    servoY.write(90);
    fireServo.write(0);

    // Set motor control pins (H-Bridge 1)
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);

    // Set firing mechanism
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(CONTROL_PIN, OUTPUT);
    digitalWrite(CONTROL_PIN, LOW);

    // Set firing motor control (H-Bridge 2)
    pinMode(FIRE_MOTOR_DIR, OUTPUT);
    pinMode(FIRE_MOTOR_PWM, OUTPUT);

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

void controlAiming() {
    int16_t x = readAxis(FXOS8700_REGISTER_OUT_X_MSB);
    int16_t z = readAxis(FXOS8700_REGISTER_OUT_X_MSB + 2);

    // Map raw sensor values to servo movement
    int targetX = map(z, SENSOR_MIN, SENSOR_MAX, SERVO_MIN, SERVO_MAX);
    int targetY = map(x, SENSOR_MIN, SENSOR_MAX, SERVO_MIN, SERVO_MAX);

    // Apply scaling factor
    targetX = 60 + (targetX - 90) * -(SCALING_FACTOR);
    targetY = 90 + (targetY - 90) * SCALING_FACTOR;

    // Smoothly interpolate using a simple low-pass filter
    filteredServoX = (SMOOTHING_FACTOR * targetX) + ((1 - SMOOTHING_FACTOR) * filteredServoX);
    filteredServoY = (SMOOTHING_FACTOR * targetY) + ((1 - SMOOTHING_FACTOR) * filteredServoY);

    // Move servos to the smoothed positions
    servoX.write(filteredServoX);
    servoY.write(filteredServoY);
}

void controlMovement() {
    int yValue = analogRead(JOYSTICK_Y_PIN);
    int center = 512;
    int deadZone = 50;

    if (yValue > center + deadZone) {
        moveForward(map(yValue, center + deadZone, 1023, 100, 255));
    } else if (yValue < center - deadZone) {
        moveBackward(map(yValue, 0, center - deadZone, 255, 100));
    } else {
        stopMotor();
    }
}

void moveForward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, speed);
}

void moveBackward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, speed);
}

void stopMotor() {
    analogWrite(MOTOR_PWM_PIN, 0);
}

void controlFiring() {
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW) {
        digitalWrite(CONTROL_PIN, LOW);
        delay(500);
        digitalWrite(CONTROL_PIN, HIGH);
    }
}

void controlFiringMotor() {
    digitalWrite(FIRE_MOTOR_DIR, HIGH);
    analogWrite(FIRE_MOTOR_PWM, 200);
}

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

void initializeSensor() {
    Wire.beginTransmission(FXOS8700_ADDR);
    Wire.write(FXOS8700_REGISTER_CTRL_REG1);
    Wire.write(0x00);  // Standby mode
    Wire.endTransmission();

    Wire.beginTransmission(FXOS8700_ADDR);
    Wire.write(FXOS8700_REGISTER_CTRL_REG1);
    Wire.write(0x0F);  // Active mode
    Wire.endTransmission();
}