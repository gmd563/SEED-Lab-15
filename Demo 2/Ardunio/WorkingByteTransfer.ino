#include <Wire.h>

#define MY_ADDR 8

void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  Serial.println("Arduino ready");
}

void loop() {
  delay(100);
}

void receiveEvent(int numBytes) {
  Serial.print("Received ");
  Serial.print(numBytes);
  Serial.println(" bytes.");

while (Wire.available()) {
  char c = Wire.read();
  Serial.print("Data: ");
  Serial.println(c, HEX);
  }
}