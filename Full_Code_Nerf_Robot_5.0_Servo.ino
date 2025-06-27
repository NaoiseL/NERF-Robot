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
#define FIRE_SERVO_PIN 4    // Servo for firing trigger

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
    Serial.println("Initialising...");
    Wire.begin();

    // Initialize FXOS8700 Sensor (I2C)
    initialiseSensor();

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

    Serial.println("Setup complete.");
}

void loop() {
  // Run the aiming control evert "updateInterval" milliseconds
    if (millis() - lastUpdateTime >= updateInterval) {
        controlAiming();
        lastUpdateTime = millis();
    }
    controlMovement(); //checks joystick and controls movement
    controlFiring(); // Check if button is pressed and fires
    delay(10);
}

// Controls the aiming system based on accelerometer readdings
void controlAiming() {
    int16_t x = readAxis(FXOS8700_REGISTER_OUT_X_MSB);
    int16_t z = readAxis(FXOS8700_REGISTER_OUT_X_MSB + 2);

    // Map raw sensor values to servo movement
    int targetX = map(z, SENSOR_MIN, SENSOR_MAX, SERVO_MIN, SERVO_MAX);
    int targetY = map(x, SENSOR_MIN, SENSOR_MAX, SERVO_MIN, SERVO_MAX);

    // Apply scaling factor to reduce sensitivity
    targetX = 10 + (targetX - 90) * -(SCALING_FACTOR);
    targetY = 30 + (targetY - 90) * -(SCALING_FACTOR);

    // Smoothly interpolate using a simple low-pass filter
    filteredServoX = (SMOOTHING_FACTOR * targetX) + ((1 - SMOOTHING_FACTOR) * filteredServoX);
    filteredServoY = (SMOOTHING_FACTOR * targetY) + ((1 - SMOOTHING_FACTOR) * filteredServoY);

    // Move servos to the smoothed positions
    servoX.write(filteredServoX);
    servoY.write(filteredServoY);

    // Debugging output
    Serial.print("Aiming: X=");
    Serial.print(filteredServoX);
    Serial.print(", Y=");
    Serial.println(filteredServoY);
}

// Controls movement based on joystick input
void controlMovement() {
    int yValue = analogRead(JOYSTICK_Y_PIN); // Reads joystick Y-axis value
    int center = 512; // Center value for joystick
    int deadZone = 100; // Dead zone to prevent unwanted movement

    if (yValue > center + deadZone) {
      Serial.println("Moving Forward");
        moveForward(map(yValue, center + deadZone, 1023, 100, 255));
    } else if (yValue < center - deadZone) {
      Serial.println("Moving Backward");
        moveBackward(map(yValue, 0, center - deadZone, 255, 100));
    } else {
      Serial.println("Stopping");
        stopMotor(); // Stop motor if in deadzone
    }
}

// Move motor forward at set speed
void moveForward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, speed);
}

// Move motor backward at set speed
void moveBackward(int speed) {
    digitalWrite(MOTOR_DIR_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, speed);
}

// stops motor
void stopMotor() {
    analogWrite(MOTOR_PWM_PIN, 0);
}

// Controls firing mechanism when the fire button is pressed
void controlFiring() {
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW) { // button is pressed
      Serial.println("Firing");
        fireServo.write(0);
        delay(300); // delay for servo movemtn
        fireServo.write(90); // move back to original position
    }
}

// Reads an axis value from the accelerometer
int16_t readAxis(uint8_t msbReg) {
    int16_t axisValue = 0; // stores final axis reading
    // begin communication with sensor
    Wire.beginTransmission(FXOS8700_ADDR);
    Wire.write(msbReg); // send register address from which to read data
    Wire.endTransmission(false); // end transmission but keep I2C active
    Wire.requestFrom(FXOS8700_ADDR, 2); // requests two bytes

    if (Wire.available() >= 2) { // checks if at keast 2 bytes are available 
        // Read first byte and shifts it 8 bits to left 
        // Read second byte and combine it with first to get a 16 bit value
        axisValue = (Wire.read() << 8) | Wire.read();
    }
    return axisValue; // return the final processed sensor value
}

// initialises the FXOS8700 accelerometer sensor
void initialiseSensor() {
  // Put sensor in standby mode to allow configuration
    Serial.println("Initialising sensor..."); 
    Wire.beginTransmission(FXOS8700_ADDR); // Begin I2C communication with sensor
    Wire.write(FXOS8700_REGISTER_CTRL_REG1); // select control register 1
    Wire.write(0x00);  // Standby mode
    Wire.endTransmission();

    // Put sensor into active mode to begin data collection
    Wire.beginTransmission(FXOS8700_ADDR); // begin new transmission
    Wire.write(FXOS8700_REGISTER_CTRL_REG1); // Select control register 1 again
    Wire.write(0x0F);  // Active mode
    Wire.endTransmission(); // end transmission
    Serial.println("Sensor Initialised.");
}
