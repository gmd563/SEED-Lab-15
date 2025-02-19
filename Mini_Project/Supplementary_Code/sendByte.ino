# Sophia Mimlitz
# Basic computer vision code required for the Arduino
# Receives a byte from the Raspberry Pi


#include <Wire.h>
#define MY_ADDR 8

volatile uint8_t offset = 0;
volatile uint8_t instruction = 0;

# Initialize (Wire, ISR)
void setup() {
	Serial.begin(115200);
	Wire.begin(MY_ADDR);
	Wire.onReceive(receive);
}

void loop() {

}

# Receive ISR, receives a byte from the Raspberry Pi via I2C
void receive() {
	offset = Wire.read();
	while(Wire.available()) {
		instruction = Wire.read();
	}
}
