import queue
import threading
from time import sleep

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


q = queue.Queue()


def myFunction():
	##########################################################
	# Initialize LCD here
	lcdCols = 16
	lcdRows = 2
	i2c = board.I2C()
	lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcdCols, lcdRows)
	lcd.clear()
	lcd.color = [0, 0, 100]
	lcd.text_direction = lcd.LEFT_TO_RIGHT
	##########################################################

	while True:
		if not q.empty():
			gotSomething = q.get()
			print(“I got: {}”.format(gotSomething))
			# Write new data to the LCD here


myThread = threading.Thread(target = myFunction, args = ())
myThread.start()


while True:

	# Send it to the thread
	if True: # Put your own conditional here (i.e. ArUco marker moved)
		q.put(..........)

	# Carry on…
	sleep(1) # I wouldn’t keep this here outside of this example

