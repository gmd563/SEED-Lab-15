# Grace Davis
# compare color detection b/w this and github and make sure that color can be updated as new markers are identified

# import all libraries
import cv2
import numpy as np
from cv2 import aruco
import queue
import threading
import time
import board
#from adafruit_character_lcd.character_lcd_rgb_i2c import Character_LCD_RGB_I2C
import smbus
import struct
# import to ease robot to stop - CAN BE TAKEN OUT
from math import tanh

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
#state = "start"

#data = []

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
        print("WA")
        time.sleep(0.01)

def send_to_i2c(command):
    try:
        i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
        return True
    except OSError:
        return False


##################################################################################
# activity[0] -> ids, activity[1] -> angle, activity[2] -> distance, activity[3] -> color

# function for easing robot to stop - CAN BE TAKEN OUT
#def ease_to_stop(target_dist):
    #return max(0, (target_dist - 1.0) * tanh(target_dist - 1.0))

activity_lock = threading.Lock() # lock shared data

# State Machine Class
class StateMachine:
    def __init__(self):
        self.ids = []
        self.state = self.start
    
    def run(self):
        while True:
            with activity_lock:
                self.state = self.state(activity)
            time.sleep(0.1) # MAY NEED TO ASJUST (according to chat)

    def start(self, activity):
        return self.search 
    
    def search(self, activity):
        if activity[0] is not None:
            send_array([1, 0, 0])
            print("moving to found")
            return self.found
        else:
            send_array([0, 0, 0])
            return self.search

    def found(self, activity):
        if activity[1] is not None:
            send_array([1, activity[1], 0])
            if abs(activity[1]) <= 30:
                print("move to move")
                return self.move
        return self.found

    def move(self, activity):
        if activity[1] is not None and activity[2] is not None: # addition of activity[2] is not None to prevent type errors if no aruco marker is detected
            if activity[2] > 1.0:
                target_dist = activity[2] - 1
                #target_dist = ease_to_stop(activity[2]) # for gradual stop - CAN BE REVERTED TO O.G. ^

                # if statement intended to bring robot to gradual stop - CAN BE TAKEN OUT
                #if activity[2] < 2.0: 
                    #target_dist *= 0.5 
                
                send_array([1, activity[1], target_dist])
                return self.move
            else:
                send_array([1, 0, 0]) # MAYBE COMMENT OUT LATER
                return self.arrived
        return self.move

    def arrived(self, activity):
        send_array([1, 0, 0])
        time.sleep(1)
        print("move to turn")
        print(activity[2],  self.ids)
        return self.turn

    def turn(self, activity):
        if activity[3] == 1:
            send_array([0, 90, 0])
            time.sleep(4)
            self.ids = []
            activity[3] = 0
            return self.search
        elif activity[3] == 2:
            send_array([0, -90, 0])
            time.sleep(4)
            self.ids = []
            return self.search
        else:
            send_array([1, 0, 0])
            return self.turn

# initialize State Machine thread
with activity_lock:
    activity = [None, None, None, None]
sm = StateMachine()
sm_thread = threading.Thread(target=sm.run, daemon=True)
sm_thread.start()

##################################################################################

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

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    #closest_marker = None
    min_distance = float('inf')
    selected_angle = None
    selected_id = None
    selected_corners = None # for croping for color detection

    height, width, _ = frame.shape
    mid_x, mid_y = round(width / 2), round(height / 2)

    angle = None  # Reset detected angle
    distance = None

    if ids is not None:
        ids = ids.flatten()
        #closest_marker = None
        #min_distance = float('inf')
        #selected_angle = None
        #selected_id = None
        for (corner, marker_id) in zip(corners, ids):
            marker_corners = corner.reshape((4, 2)).astype(np.int32)
            #center_x = int(marker_corners[:, 0].mean())
            #center_y = int(marker_corners[:, 1].mean())

            # compute distance in ft
            marker_width_px = np.linalg.norm(marker_corners[0] - marker_corners[1])
            marker_height_px = np.linalg.norm(marker_corners[1] - marker_corners[2])
            marker_size_px = (marker_width_px + marker_height_px) / 2

            focal_length = camera_matrix[0, 0]
            distance = ((marker_size * focal_length) / marker_size_px) * 3.28084

            if distance <= 5.0 and distance < min_distance:
                # compute viewing angle
                center_x = int(marker_corners[:, 0].mean())
                center_y = int(marker_corners[:, 1].mean())
                offset_x = center_x - mid_x

                min_distance = distance
                selected_angle = -np.degrees(np.arctan2(offset_x, focal_length))
                angle = selected_angle
                selected_id = marker_id
                selected_corners = marker_corners
                
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

            #if selected_id is not None:
                #with activity_lock:
                    #activity = [selected_id, selected_angle, min_distance, color]
            #else:
                #with activity_lock:
                    #activity[:] = [None, None, None, None]
            with activity_lock:
                activity = [selected_id, selected_angle, min_distance, color]
            
        else:
            with activity_lock:
                activity[:] = [None, None, None, None]
                
    else:
        angle = "No ArUco Marker"
        with activity_lock:
            activity = [None, None, None, None]

    ##########################################################################
    #current_state = current_state(activity)

    if angle != last_angle:
        q.queue.clear()
        q.put(f"Angle: {angle:.2f} deg" if isinstance(angle, (int, float)) else angle)
        last_angle = angle

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
