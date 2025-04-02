# import all libraries
import cv2
import numpy as np
from cv2 import aruco
import queue
import threading
from time import sleep
import board
import adafruit_character_lcd.character_lcd
from smbus2 import SMBus
import struct

# initialize Arduino i2c
ARD_ADDR = 8
i2c1 = SMBus(1)
offset = 1

# initialize camera
camera = cv2.VideoCapture(0)

# load Aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

# load camera calibration data
calibration_data = np.load('camera_calibration_data.npz')
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

# set size of Aruco marker in meters
marker_size = 0.05

# camera's horizontal field of view (FOV) in degrees
camera_fov = 61 ###########################

# queue for LCD
q = queue.Queue(maxsize = 1)

# store last displayed angle
last_angle = None

# initialize current state
current_state = None

data = []
activity = []

def start(activity):
	return search

def search(activity):
	if activity[0] is not None:
		data = [1, 0, 0]
		command = [ ]
		for value in command:
			command.extend(struct.pack('<f', value))

		i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
		return found
	else:
        data = [0, 0, 0]
        command = [ ]
		for value in command:
			command.extend(struct.pack('<f', value))
		i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
        return search

def found(activity):
	data = [1, activity[1], 0]
    command = [ ]
	for value in command:
		command.extend(struct.pack('<f', value))
	i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
    if activity[1] <= 1:
		return move
	else:
		return found
\
def move(activity):
	data = [1, activity[1], activity[2] - 1]
    command = [ ]
	for value in command:
		command.extend(struct.pack('<f', value))
	i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
	if activity[2] <= 1:
		return arrived
	else:
		return move

def arrived(activity):
	sleep(0.2)
	return turn

def turn(activity):\
	if activity[3] = 1:\
		\kerning1\expnd0\expndtw0 data = [1, 90, 0]\
		command = [ ]\
		for value in command:\
			command.extend(struct.pack(\'91<f\'92, value\expnd0\expndtw0\kerning0
))\
		i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])\
		return search\
\kerning1\expnd0\expndtw0 \
	elif activity[3] = 2:\
		data = [1, -90, 0]\
		command = [ ]\
		for value in command:\
			command.extend(struct.pack(\'91<f\'92, value\expnd0\expndtw0\kerning0
))\
		i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])\
		return search\
\kerning1\expnd0\expndtw0 \
	else:\
		data = [0, 0, 0]\
		command = [ ]\
		for value in command:\
			command.extend(struct.pack(\'91<f\'92, value\expnd0\expndtw0\kerning0
))\
		i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])\
		return turn\
\
def lcd_thread():\
    lcdCols = 16\
    lcdRows = 2\
    i2c2 = board.I2C()\
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c2, lcdCols, lcdRows)\
    lcd.clear()\
    lcd.color = [0, 0, 100]\
    lcd.text_direction = lcd.LEFT_TO_RIGHT\
\
    last_message = None  # Store last message displayed\
\
    while True:\
        if not q.empty():\
            message = q.get()\
            if message != last_message:  # Only update if message changed\
                print(f"LCD displaying: \{message\}")\
                lcd.clear()\
                lcd.message = str(message)\
                last_message = message\
        sleep(0.1)  # Prevent excessive CPU usage\
\
# Start LCD thread\
myThread = threading.Thread(target=lcd_thread, daemon=True)\
myThread.start()\
\
# initialize state values\
current_state = start\
activity = [None, None, None, None]\
\
while True:\
	ret, frame = camera.read()\
	if not ret:\
		print("Failed to grab frame")\
		break\
\
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)\
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)\
\
	# Define color ranges\
	lower_green = np.array([40, 100, 50])\
	upper_green = np.array([80, 255, 255])\
	lower_red1 = np.array([0, 150, 80])\
	upper_red1 = np.array([10, 255, 255])\
	lower_red2 = np.array([170, 150, 80])\
	upper_red2 = np.array([180, 255, 255])\
\
	mask_green = cv2.inRange(hsv, lower_green, upper_green)\
	mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)\
	mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)\
	mask_red = cv2.bitwise_or(mask_red1, mask_red2)\
\
	if np.sum(mask_green) > 0:\
		color = 1  # Green detected\
	elif np.sum(mask_red) > 0:\
		color = 2  # Red detected\
	else:\
		color = 0  # No color detected\
\
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)\
\
	height, width, _ = frame.shape\
	mid_x, mid_y = round(width / 2), round(height / 2)\
\
	angle = None  # Reset detected angle\
	distance = None\
\
	if ids is not None:\
		ids = ids.flatten()\
		for (corner, marker_id) in zip(corners, ids):\
			marker_corners = corner.reshape((4, 2))\
			center_x = int(marker_corners[:, 0].mean())\
			center_y = int(marker_corners[:, 1].mean())\
\
			# Calculate the apparent size of the marker in pixels\
			marker_width_px = np.linalg.norm(marker_corners[0] - marker_corners[1])\
			marker_height_px = np.linalg.norm(marker_corners[1] - marker_corners[2])\
			marker_size_px = (marker_width_px + marker_height_px) / 2\
\
			# Estimate the distance of the marker from the camera\
			# Using the formula: distance = (known_size * focal_length) / apparent_size\
			focal_length = camera_matrix[0, 0]  # Focal length in pixels (from camera matrix)\
			distance = (marker_size * focal_length) / marker_size_px\
			distance = distance * 3.28084\
\
			# Calculate the horizontal offset from the center of the screen\
			offset_x = center_x - mid_x\
\
			# Calculate the true horizontal angle using trigonometry\
			angle = np.degrees(np.arctan2(offset_x, focal_length))\
\
			activity = [ids[0], angle, distance, color]\
			# break here?\
\
	else:\
		angle = "No ArUco Marker"\
		activity = [None, None, None, None]\
\
	current_state = current_state(activity)\
\
	# Only update LCD if angle changes (no more flashing)\
	if angle != last_angle:\
		q.queue.clear()  # Remove old data before adding new\
		q.put(f"Angle: \{angle:.2f\} deg" if isinstance(angle, (int, float)) else angle)\
		last_angle = angle  # Update last displayed angle\
\
	if 0xFF == ord('q'):\
		break\
\
camera.release()\
	\
\pard\tx429\tx863\tx1299\tx1733\tx2162\tx2590\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0
\cf0 		\
\
}
