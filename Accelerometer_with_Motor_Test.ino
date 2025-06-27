#include <Wire.h>

// FXOS8700 I2C Address
#define FXOS8700_ADDR 0x1E

// FXOS8700 Registers
#define FXOS8700_REGISTER_CTRL_REG1  0x2A
#define FXOS8700_REGISTER_OUT_X_MSB  0x01

// Motor control pins
#define M1 8   // Motor direction pin
#define E1 10  // PWM pin for speed control

void setup() {
  Serial.begin(115200);  // Set baud rate
  Wire.begin();  // Initialize I2C communication

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

  // Set motor control pins
  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
}

void loop() {
  int16_t x = readAxis(FXOS8700_REGISTER_OUT_X_MSB);

  Serial.print("X-axis: ");
  Serial.println(x);

  // Motor control logic based on X-axis tilt
  if (x > 1000) {  // Tilted forward
    moveForward();
  } 
  else if (x < -1000) {  // Tilted backward
    moveBackward();
  } 
  else {  // Near level position
    stopMotor();
  }

  delay(100);
}

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

void moveForward() {
  digitalWrite(M1, LOW);  // Forward direction
  analogWrite(E1, 250);   // Adjust speed (0-255)
  Serial.println("Moving Forward");
}

void moveBackward() {
  digitalWrite(M1, HIGH);  // Backward direction
  analogWrite(E1, 250);
  Serial.println("Moving Backward");
}

void stopMotor() {
  analogWrite(E1, 0);  // Stop motor by setting speed to 0
  Serial.println("Motor Stopped");
}
