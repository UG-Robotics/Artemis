/* Minimal I2C bus scanner. TCS34725 should appear at 0x29. */
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(500);
}

void loop() {
  int found = 0;
  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  none found (check power, GND, SDA->A4, SCL->A5)");
  Serial.println();
  delay(2000);
}
