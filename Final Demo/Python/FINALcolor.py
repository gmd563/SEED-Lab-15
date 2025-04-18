# import all libraries
import cv2
import numpy as np
from cv2 import aruco
import queue
import threading
import time
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

# arrival time
arrival_time = time.time()

def adjust_intensity(frame, alpha=1.0, beta=0):
    adjusted = np.clip(alpha * frame + beta, 0, 255).astype(np.uint8)
    return adjusted

def detect_arrow_color(mask_red, mask_green, region=None):
    def find_largest_arrow(mask):
        if region is not None:
            x1, y1, x2, y2 = region
            mask = mask[y1:y2, x1:x2]

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small stuff
                approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
                if len(approx) >= 5:  # Likely arrow shape
                    if area > largest_area:
                        largest_area = area

        return largest_area

    red_area = find_largest_arrow(mask_red)
    green_area = find_largest_arrow(mask_green)


    if red_area == 0 and green_area == 0:
        return 0  # No arrow
    elif red_area > green_area:
        return 1  # Red arrow (larger)
    elif green_area > red_area:
        return 2  # Green arrow (larger)
    else:
        return 0  # Equal size



def send_array(data):
    command = []
    for value in data:
        command.extend(struct.pack('<f', value))
    #print("Sending bytes:", [hex(b) for b in command[:32]]) 
    while not send_to_i2c(command):
        time.sleep(0.01)

def send_to_i2c(command):
    try:
        i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
        return True
    except OSError:
        return False


def start(activity):
    global state
    
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
            print(activity[2])
            return arrived
    return move

def arrived(activity):
    global arrival_time
    data = [1, 0, 0]
    send_array(data)
    time.sleep(1)
    arrival_time = time.time()
    print("startturn")
    return turn

def turn(activity):
    if activity[3] == 1:
        data = [0, 90, 0]
        send_array(data)
        return wait
    elif activity[3] == 2:
        data = [0, -90, 0]
        send_array(data)
        return wait
    else:
        data = [1, 0, 0]
        send_array(data)
        return turn

def wait(activity):
    global arrival_time
    if time.time() - arrival_time < 4:
        return wait
    return search

# Initialize state machine
current_state = start
activity = [None, None, None, None]

marker_color_history = {}  # persists across loop iterations

marker_color_history = {}  # Remember each marker's last known color

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

    

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    height, width, _ = frame.shape
    mid_x, mid_y = round(width / 2), round(height / 2)

    angle = None
    distance = None

    if ids is not None:
        ids = ids.flatten()
        selected_id = None
        selected_angle = None
        selected_distance = None
        selected_color = 0
        min_distance = float('inf')

        for (corner, marker_id) in zip(corners, ids):
            marker_corners = corner.reshape((4, 2))
            center_x = int(marker_corners[:, 0].mean())
            center_y = int(marker_corners[:, 1].mean())

            marker_width_px = np.linalg.norm(marker_corners[0] - marker_corners[1])
            marker_height_px = np.linalg.norm(marker_corners[1] - marker_corners[2])
            marker_size_px = (marker_width_px + marker_height_px) / 2

            focal_length = camera_matrix[0, 0]
            distance = (marker_size * focal_length) / marker_size_px
            distance = distance * 3.28084  # convert to feet

            
            if distance <= 5.0 and distance < min_distance:
            
                offset_x = center_x - mid_x
                angle = -np.degrees(np.arctan2(offset_x, focal_length))

                # Define region around marker center for arrow detection
                region_sizey = 1000
                region_sizex = 1000
                x1 = max(center_x - region_sizex, 0)
                x2 = min(center_x + region_sizex, width)
                y1 = max(center_y - region_sizey, 0)
                y2 = min(center_y + region_sizey, height)
                region = (x1, y1, x2, y2)

                if detect_arrow_color(mask_red, mask_green, region=region) == 2:
                    marker_color = 2
                if detect_arrow_color(mask_red, mask_green, region=region) == 1:
                    marker_color = 1
                else:
                    marker_color = marker_color_history.get(marker_id, 0)  # fallback to last known color

                marker_color_history[marker_id] = marker_color

                selected_id = marker_id
                selected_angle = angle
                selected_distance = distance
                selected_color = marker_color
                min_distance = distance

        # ROI Mask ###############################################################
        if selected_id is not None:
            rect = cv2.minAreaRect(selected_corners) # Rotate Rectangle
            (crop_x, crop_y), (w, h), crop_angle = rect
            #rect_points = np.int0(cv2.boxPoints(rect)) # Identify corners

                #w = int(rect[1][0]) # Rotate Rectangle w/ perspective transform
                #h = int(rect[1][1]) # Rotate Rectangle w/ perspective transform

            if w < h:  # Ensure correct width
                w,h = h,w # swap width and height to ensure ROI is always wider than tall
            extend_w = int(1.5*w)

            box = cv2.boxPoints(((crop_x, crop_y), (extend_w, int(h)), crop_angle))

            #rect = (rect[0], 1.5 * rect[1][0], rect[1][1], rect[2]) # Increase ROI Area in width
            destination_pts = np.array([[0,0], [extend_w - 1, 0], [extend_w - 1, h - 1], [0, h - 1]], dtype="float32") # Define Destination Points
            perspective_mtx = cv2.getPerspectiveTransform(box.astype("float32"), destination_pts) # Perspective Transform Matrix
            crop_hsv = cv2.warpPerspective(hsv, perspective_mtx, (extend_w,int(h))) # Crop Image

            # Implement ROI in color detection
            mask_green = cv2.inRange(crop_hsv, lower_green, upper_green)
            mask_red1 = cv2.inRange(crop_hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(crop_hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # CAN REMOVE - noise removal for color detection
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, k)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, k)

            # arrow detection  
            red_arrow_detected = detect_arrow_color(mask_red, "red")
            green_arrow_detected = detect_arrow_color(mask_green, "green")

            if red_arrow_detected and not green_arrow_detected:
                color = 2  # Red arrow detected
            elif green_arrow_detected and not red_arrow_detected:
                color = 1  # Green arrow detected
            else:
                color = 0  # Default or "none"

        if selected_id is not None and selected_distance is not None:
            activity[:] = [ids, selected_angle, selected_distance, selected_color]
        else:
            activity[:] = [None, None, None, None]
    else:
        activity[:] = [None, None, None, None]


    current_state = current_state(activity)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
