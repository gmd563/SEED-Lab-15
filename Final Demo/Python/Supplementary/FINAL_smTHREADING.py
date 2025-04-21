# import all libraries
import cv2
import numpy as np
from cv2 import aruco
import queue
import threading
import time
import board
import smbus
import struct

# initialize Arduino i2c
ARD_ADDR = 8
i2c1 = smbus.SMBus(1)
offset = 1

# initialize camera
camera = cv2.VideoCapture(0)

# load ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

# load camera calibration data
calibration_data = np.load('camera_calibration_data.npz')
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

# set size of ArUco marker in meters
marker_size = 0.05

# camera's horizontal field of view (FOV) in degrees
camera_fov = 61

state = "start"
data = []
alpha = 1.5  # Contrast control
beta = 20    # Brightness control
arrival_time = time.time()
marker_color_history = {}

# adjust brightness and contrast
def adjust_intensity(frame, alpha=1.0, beta=0):
    adjusted = np.clip(alpha * frame + beta, 0, 255).astype(np.uint8)
    return adjusted

def send_array(data):
    command = []
    for value in data:
        command.extend(struct.pack('<f', value))
    while not send_to_i2c(command):
        time.sleep(0.01)

def send_to_i2c(command):
    try:
        i2c1.write_i2c_block_data(ARD_ADDR, offset, command[:32])
        return True
    except OSError:
        return False

##################################################################################
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
            data = [1, 0, 0]
            send_array(data)
            #print("moving to found")
            return self.found
        else:
            data = [0, 0, 0]
            send_array(data)
            return self.search

    def found(self, activity):
        if activity[1] is not None:
            data = [1, activity[1], 0]
            send_array(data)
            if abs(activity[1]) <= 30:
                #print("move to move")
                return self.move
        return self.found

    def move(self, activity):
        if activity[1] is not None and activity[2] is not None: # addition of activity[2] is not None to prevent type errors if no aruco marker is detected
            if activity[2] > 1.2: #### FLAG - CHANGE FOR FINE TUNED CONTROL ####
                data = [1, activity[1], activity[2] - 1]
                send_array(data)
                return self.move
            else:
                return self.arrived
        return self.move

    def arrived(self, activity):
        global arrival_time
        data = [1, 0, 0]
        send_array(data)
        time.sleep(1)
        arrival_time = time.time()
        return self.turn

    def turn(self, activity):
        if activity[3] == 1:
            data = [0, 90, 0]
            send_array(data)
            return self.wait
        elif activity[3] == 2:
            data = [0, -90, 0]
            send_array(data)
            return self.wait
        else:
            data = [1, 0, 0]
            send_array(data)
            return self.turn
        
    def wait(self, activity):
        global arrival_time
        if time.time() - arrival_time < 4:
            return self.wait
        return self.search

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

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    height, width, _ = frame.shape
    mid_x = width // 2

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
            distance *= 3.28084

            if distance <= 5.7 and distance < min_distance: #### FLAG - CHANGE FOR FINE TUNED CONTROL ####
                offset_x = center_x - mid_x
                angle = -np.degrees(np.arctan2(offset_x, focal_length))

                region_width = int(marker_width_px * 3)
                region_height = int(marker_height_px)
                x1 = max(center_x - region_width // 2, 0)
                x2 = min(center_x + region_width // 2, width)
                y1 = max(center_y - region_height // 2, 0)
                y2 = min(center_y + region_height // 2, height)

                region_red = mask_red[y1:y2, x1:x2]
                region_green = mask_green[y1:y2, x1:x2]

                red_sum = np.sum(region_red)
                green_sum = np.sum(region_green)

                if red_sum > green_sum and red_sum > 50000:
                    marker_color = 2
                elif green_sum > red_sum and green_sum > 50000:
                    marker_color = 1
                else:
                    marker_color = 0

                selected_id = marker_id
                selected_angle = angle
                selected_distance = distance
                selected_color = marker_color
                min_distance = distance

        if selected_id is not None and selected_distance is not None:
            activity[:] = [ids, selected_angle, selected_distance, selected_color]
        else:
            activity[:] = [None, None, None, None]
    else:
        activity[:] = [None, None, None, None]

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
