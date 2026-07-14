/*
 * imu_test.ino  —  MPU6050 (GY-521 / 10-DOF) test on an Arduino Uno
 * ---------------------------------------------------------------------------
 * Streams WHO_AM_I, accel (g), gyro (deg/s), and temperature (C) to Serial.
 * Hot-detects the sensor: if it's not found it prints "waiting..." and keeps
 * retrying, so you can wire it live and watch it come alive.
 *
 * ===========================  WIRING  ======================================
 *   MPU6050 pin  ->  Uno pin      Notes
 *   ---------------------------------------------------------------
 *   VCC          ->  5V           GY-521 has an onboard regulator (3-5V OK)
 *   GND          ->  GND          common ground
 *   SDA          ->  A4           I2C data  (Uno hardware I2C)
 *   SCL          ->  A5           I2C clock (Uno hardware I2C)
 *   AD0          ->  GND (or n/c) low = address 0x68 (high = 0x69)
 *   INT, XCL, XDA->  (leave unconnected)
 *
 * Serial monitor: 115200 baud.
 * ---------------------------------------------------------------------------
 */

#include <Wire.h>

const uint8_t MPU = 0x68;          // AD0 low. Change to 0x69 if AD0 is high.
const uint8_t WHO_AM_I   = 0x75;
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

const float ACCEL_SENS = 16384.0;  // LSB/g   at +/-2g  (default)
const float GYRO_SENS  = 131.0;    // LSB/dps at +/-250 (default)

bool inited = false;

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;   // NACK -> nothing there
  Wire.requestFrom(MPU, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool tryInit() {
  uint8_t who = readReg(WHO_AM_I);
  Serial.print("WHO_AM_I(0x75) = 0x"); Serial.println(who, HEX);
  if (who == 0xFF) return false;                  // nothing responding on the bus
  if (who != 0x68 && who != 0x69)                 // 0x68 = genuine MPU6050
    Serial.println("  note: nonstandard ID (likely an MPU6050 clone) - trying anyway");
  writeReg(PWR_MGMT_1, 0x00);   // wake from sleep
  delay(50);
  Serial.println("device woken — streaming (expect ~1g at rest, gyro ~0):");
  return true;
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);   // 25ms; auto-reset TWI so a stuck bus can't hang us
  Serial.begin(115200);
  delay(300);
  Serial.println("MPU6050 test — Uno");
}

void loop() {
  if (!inited) {
    inited = tryInit();
    if (!inited) { Serial.println("  waiting for MPU6050 (check wiring)..."); delay(600); }
    return;
  }

  // burst-read 14 bytes: ax,ay,az, temp, gx,gy,gz  (each int16, big-endian)
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) { Serial.println("  read failed - lost the sensor?"); inited = false; return; }
  Wire.requestFrom(MPU, (uint8_t)14);
  if (Wire.available() < 14) { Serial.println("  short read"); delay(200); return; }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t traw = (Wire.read() << 8) | Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  float tempC = traw / 340.0 + 36.53;

  Serial.print("A[g] ");
  Serial.print(ax / ACCEL_SENS, 2); Serial.print(", ");
  Serial.print(ay / ACCEL_SENS, 2); Serial.print(", ");
  Serial.print(az / ACCEL_SENS, 2);
  Serial.print("  G[dps] ");
  Serial.print(gx / GYRO_SENS, 1); Serial.print(", ");
  Serial.print(gy / GYRO_SENS, 1); Serial.print(", ");
  Serial.print(gz / GYRO_SENS, 1);
  Serial.print("  T ");
  Serial.print(tempC, 1); Serial.println("C");

  delay(200);
}
