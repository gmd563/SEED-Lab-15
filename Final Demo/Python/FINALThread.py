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

# Intensity calibration variables
alpha = 1.5  # Contrast control
beta = 20    # Brightness control

def adjust_intensity(frame, alpha=1.0, beta=0):
    adjusted = np.clip(alpha * frame + beta, 0, 255).astype(np.uint8)
    return adjusted

def detect_arrow_color(mask, color_name):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    arrow_like_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Ignore small stuff
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
            if len(approx) >= 5:  # Arrows often have 5+ points
                arrow_like_contours.append((area, contour))

    if arrow_like_contours:
        arrow_like_contours.sort(reverse=True)  # Largest contour first
        return True  # Arrow of this color detected
    return False

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

# Initialize state machine
current_state = start
activity_lock = threading.Lock()
activity = [None, None, None, None]

def get_latest_activity():
    with activity_lock:
        return activity.copy()

class StateMachineThread(threading.Thread):
    def __init__(self, get_activity_func):
        super().__init__()
        self.get_activity = get_activity_func
        self.current_state = start
        self.running = True
        self.turn_start_time = None

    def run(self):
        while self.running:
            activity = self.get_activity()
            self.current_state = self.current_state(activity)
            sleep(0.05)  # Prevent CPU overload

    def stop(self):
        self.running = False

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

# Start State Machine Thread
state_machine = StateMachineThread(get_latest_activity)
state_machine.start()


while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    adjusted_frame = adjust_intensity(frame, alpha, beta)

    gray = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2HSV)

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

    red_arrow_detected = detect_arrow_color(mask_red, "red")
    green_arrow_detected = detect_arrow_color(mask_green, "green")
    
    if red_arrow_detected and not green_arrow_detected:
        color = 2  # Red arrow detected
    elif green_arrow_detected and not red_arrow_detected:
        color = 1  # Green arrow detected
    else:
        color = 0  # Default or "none"

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    height, width, _ = frame.shape
    mid_x, mid_y = round(width / 2), round(height / 2)

    angle = None  # Reset detected angle
    distance = None

    if ids is not None:
        ids = ids.flatten()
        closest_marker = None
        min_distance = float('inf')
        selected_angle = None
        selected_id = None
        
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

            if distance <= 5.0 and distance < min_distance:
                min_distance = distance
                offset_x = center_x - mid_x
                selected_angle = -np.degrees(np.arctan2(offset_x, focal_length))
                selected_id = marker_id

            if selected_id is not None:
                with activity_lock:
                    activity = [selected_id, selected_angle, min_distance, color]

                
    else:
        angle = "No ArUco Marker"
        with activity_lock:
            activity = [None, None, None, None]


    current_state = current_state(activity)

    if angle != last_angle:
        q.queue.clear()
        q.put(f"Angle: {angle:.2f} deg" if isinstance(angle, (int, float)) else angle)
        last_angle = angle

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

state_machine.stop()
state_machine.join()
camera.release()
