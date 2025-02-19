# Sophia Mimlitz
# Basic Python code required to send a byte to the Arduino

from smbus2 import SMBus
from time import sleep

# Initialize
ARD_ADDR = 8
i2c = SMBus(1)
offset = 1

while(True):
	# change value of command to be 0, 1, 2, 3 (represents 00, 01, 10, 11)
	command = 0
	# Line that writes a byte (command) from  Pi to Arduino
	i2c.write_byte_data(ARD_ADDR, offset, command)
