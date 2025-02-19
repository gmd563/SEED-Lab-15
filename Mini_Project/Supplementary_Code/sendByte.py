from smbus2 import SMBus
from time import sleep

ARD_ADDR = 8
i2c = SMBus(1)
offset = 1

while(True):
	# change value of command to be 0, 1, 2, 3 (represents 00, 01, 10, 11)
	command = 0
	i2c.write_byte_data(ARD_ADDR, offset, command)
