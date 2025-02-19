#include <Wire.h>
#define MY_ADDR 8

volatile uint8_t offset = 0;
volatile uint8_t instruction = 0;

void setup() {
	Serial.begin(115200);
	Wire.begin(MY_ADDR);
	Wire.onReceive(receive);
}

void loop() {

}

void receive() {
	offset = Wire.read();
	while(Wire.available()) {
		instruction = Wire.read();
	}
}
