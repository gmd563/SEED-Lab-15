# import all libraries
import cv2
import numpy as np
from cv2 import aruco
import queue
import threading
from time import sleep
import board
from adafruit_character_lcd.character_lcd_rgb_i2c import Character_LCD_RGB_I2C
import smbus
import struct

# initialize Arduino i2c
ARD_ADDR = 8
i2c1 = smbus.SMBus(1)
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
camera_fov = 61

# queue for LCD
q = queue.Queue(maxsize=1)

# store last displayed angle
last_angle = None

# Global state variables
state = "start"

data = []

def send_array(data):
    command = []
    for value in data:
        command.extend(struct.pack('<f', value))
    #print("Sending bytes:", [hex(b) for b in command[:32]]) 
    while not send_to_i2c(command):
        print("Waiting for Arduino to be ready...")
        sleep(0.1)


def send_to_i2c(command):
    try:
        i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
        return True
    except OSError:
        return False


def start(activity):
    global state
    print("search")
    return search  # Returns a function reference


def search(activity):
    if activity[0] is not None:
        data = [1, 0, 0]
        send_array(data)
        return found
    else:
        data = [0, 0, 0]
        send_array(data)
        return search

def found(activity):
    if activity[1] is not None:
        data = [1, activity[1], 0]
        send_array(data)
        if abs(activity[1]) <= 30:
            return move
    return found

def move(activity):
    if activity[1] is not None:
        data = [1, activity[1], activity[2] - 1]
        send_array(data)
        if activity[2] <= 1:
            return arrived
    return move

def arrived(activity):
    data = [1, 0, 0]
    send_array(data)
    sleep(2)
    return turn

def turn(activity):
    if activity[3] == 1:
        data = [0, 90, 0]
        send_array(data)
        sleep(4)
        return search
    elif activity[3] == 2:
        data = [0, -90, 0]
        send_array(data)
        sleep(4)
        return search
    else:
        data = [1, 0, 0]
        send_array(data)
        return turn

def lcd_thread():
    lcdCols = 16
    lcdRows = 2
    i2c2 = board.I2C()
    lcd = Character_LCD_RGB_I2C(i2c2, lcdCols, lcdRows)
    lcd.clear()
    lcd.color = [0, 0, 100]
    lcd.text_direction = lcd.LEFT_TO_RIGHT

    last_message = None  # Store last message displayed

    while True:
        if not q.empty():
            message = q.get()
            if message != last_message:  # Only update if message changed
                print(f"LCD displaying: {message}")
                lcd.clear()
                lcd.message = str(message)
                last_message = message
        sleep(0.1)  # Prevent excessive CPU usage

# Start LCD thread
myThread = threading.Thread(target=lcd_thread, daemon=True)
myThread.start()

# Initialize state machine
current_state = start
activity = [None, None, None, None]

while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color ranges
    lower_green = np.array([40, 100, 50])
    upper_green = np.array([80, 255, 255])
    lower_red1 = np.array([0, 150, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 80])
    upper_red2 = np.array([180, 255, 255])

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)


    if np.sum(mask_red) > np.sum(mask_green):
        color = 2  # Red detected
    elif np.sum(mask_green) > np.sum(mask_red):
        color = 1  # Green detected
    else:
        color = 1  # No color detected

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    height, width, _ = frame.shape
    mid_x, mid_y = round(width / 2), round(height / 2)

    angle = None  # Reset detected angle
    distance = None

    if ids is not None:
        ids = ids.flatten()
        for (corner, marker_id) in zip(corners, ids):
            marker_corners = corner.reshape((4, 2))
            center_x = int(marker_corners[:, 0].mean())
            center_y = int(marker_corners[:, 1].mean())

            marker_width_px = np.linalg.norm(marker_corners[0] - marker_corners[1])
            marker_height_px = np.linalg.norm(marker_corners[1] - marker_corners[2])
            marker_size_px = (marker_width_px + marker_height_px) / 2

            focal_length = camera_matrix[0, 0]
            distance = (marker_size * focal_length) / marker_size_px
            distance = distance * 3.28084

            offset_x = center_x - mid_x
            angle = -np.degrees(np.arctan2(offset_x, focal_length))

            activity = [ids[0], angle, distance, color]
    else:
        angle = "No ArUco Marker"
        activity = [None, None, None, None]

    current_state = current_state(activity)

    if angle != last_angle:
        q.queue.clear()
        q.put(f"Angle: {angle:.2f} deg" if isinstance(angle, (int, float)) else angle)
        last_angle = angle

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
