#include <Wire.h>
#include <Servo.h>
#include <math.h>

// FXOS8700 I2C Address
#define FXOS8700_ADDR 0x1E

// FXOS8700 Registers
#define FXOS8700_REGISTER_CTRL_REG1  0x2A
#define FXOS8700_REGISTER_OUT_X_MSB  0x01
#define FXOS8700_REGISTER_OUT_Y_MSB  0x03  // Y-Axis Magnetometer Register

// Servo control pins
#define SERVO_X_PIN 9  // Servo for Twisting (Yaw)
#define SERVO_Y_PIN 10 // Servo for Tilting (Pitch)

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize FXOS8700 sensor
  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_CTRL_REG1);
  Wire.write(0x00);  // Standby mode
  Wire.endTransmission();

  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_CTRL_REG1);
  Wire.write(0x0F);  // Active mode
  Wire.endTransmission();

  Serial.println("FXOS8700 Sensor Initialized");

  // Attach servos
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Set servos to neutral position (90 degrees)
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  int16_t x = readAxis(FXOS8700_REGISTER_OUT_X_MSB);       // Roll (Side Tilt)
  int16_t y = readAxis(FXOS8700_REGISTER_OUT_Y_MSB);       // Pitch (Forward/Backward Tilt)

  // Compute yaw (twisting) using atan2 function
  float yaw = atan2(y, x) * (180.0 / M_PI);

  Serial.print("X-axis (Tilt): ");
  Serial.print(x);
  Serial.print(" | Y-axis (Tilt): ");
  Serial.print(y);
  Serial.print(" | Computed Yaw (Twist): ");
  Serial.println(yaw);

  // Map computed yaw to control X servo
  int servoXAngle = map(yaw, -90, 90, 30, 150);

  // Map X-axis (Tilt) to control Y servo
  int servoYAngle = map(x, -12000, 12000, 30, 150);

  // Move the servos
  servoX.write(servoXAngle);
  servoY.write(servoYAngle);

  delay(50);
}

// Read a 16-bit value from FXOS8700
int16_t readAxis(uint8_t msbReg) {
  int16_t axisValue = 0;

  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(msbReg);
  Wire.endTransmission(false);  // Repeated start
  Wire.requestFrom(FXOS8700_ADDR, 2);

  if (Wire.available() >= 2) {
    axisValue = (Wire.read() << 8) | Wire.read();
  }

  return axisValue;
}
