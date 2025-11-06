#include <Wire.h>

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
      delay(10);
    }
  }
}