#include <Wire.h>

// FXOS8700 I2C Address
#define FXOS8700_ADDR 0x1E

// FXOS8700 Registers
#define FXOS8700_REGISTER_CTRL_REG1  0x2A
#define FXOS8700_REGISTER_OUT_X_MSB  0x01
#define FXOS8700_REGISTER_OUT_X_LSB  0x02
#define FXOS8700_REGISTER_OUT_Y_MSB  0x03
#define FXOS8700_REGISTER_OUT_Y_LSB  0x04
#define FXOS8700_REGISTER_OUT_Z_MSB  0x05
#define FXOS8700_REGISTER_OUT_Z_LSB  0x06

void setup() {
  Serial.begin(115200);  // Set baud rate to 115200
  Wire.begin();  // Initialize I2C communication

  // Initialize the FXOS8700 sensor by setting CTRL_REG1 to 0x0 (standby mode)
  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_CTRL_REG1);
  Wire.write(0x00);  // Standby mode
  Wire.endTransmission();

  // Set CTRL_REG1 to 0x0F to enable the sensor (active mode)
  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_CTRL_REG1);
  Wire.write(0x0F);  // Active mode
  Wire.endTransmission();
  
  Serial.println("FXOS8700 Sensor Initialized");
}

void loop() {
  // Read accelerometer data for X, Y, and Z axes

  int16_t x, y, z;

  // Read X-axis data
  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_OUT_X_MSB);
  Wire.endTransmission(false);  // Repeated start
  Wire.requestFrom(FXOS8700_ADDR, 2);
  if (Wire.available()) {
    x = Wire.read() << 8 | Wire.read();  // Read 2 bytes (MSB and LSB)
  }

  // Read Y-axis data
  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_OUT_Y_MSB);
  Wire.endTransmission(false);  // Repeated start
  Wire.requestFrom(FXOS8700_ADDR, 2);
  if (Wire.available()) {
    y = Wire.read() << 8 | Wire.read();  // Read 2 bytes (MSB and LSB)
  }

  // Read Z-axis data
  Wire.beginTransmission(FXOS8700_ADDR);
  Wire.write(FXOS8700_REGISTER_OUT_Z_MSB);
  Wire.endTransmission(false);  // Repeated start
  Wire.requestFrom(FXOS8700_ADDR, 2);
  if (Wire.available()) {
    z = Wire.read() << 8 | Wire.read();  // Read 2 bytes (MSB and LSB)
  }

  // Print out the accelerometer readings
  Serial.print("X: "); Serial.print(x);
  Serial.print("  Y: "); Serial.print(y);
  Serial.print("  Z: "); Serial.println(z);

  delay(100);  // Wait for 500ms before reading again
}
