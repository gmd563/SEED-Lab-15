#sends floats to arduino

import smbus
from time import sleep
import struct

bus = smbus.SMBus(1)
i2c_addr = 8
register = 1

float_values = [1, 0.4, 0.99]

byte_data = []
for value in float_values:
    byte_data.extend(struct.pack('<f', value))

print("Sending bytes:", [hex(b) for b in byte_data])

bus.write_i2c_block_data(i2c_addr, register, byte_data[:32])

#wait for aruco marker to be detected

#once detected, send info to arduino to tell it to stop spinning
#also start sending angle and distance data to arduino

