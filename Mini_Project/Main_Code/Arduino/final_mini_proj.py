# Ryan Cockrill
# Mix it all together (the ArUco detection, LCD, and sending to Arduino)

#imports
import cv2
import numpy as np
from cv2 import aruco
import queue
import threading
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus

# Initialize Arduino I2C
ARD_ADDR = 8
i2c1 = SMBus(1)
offset = 1

# Initialize camera
camera = cv2.VideoCapture(0)

# Load ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

q = queue.Queue(maxsize=1)  # Queue for LCD updates

# Store last displayed quadrant
last_quadrant = None  

def lcd_thread():
    lcdCols = 16
    lcdRows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcdCols, lcdRows)
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

while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    height, width, _ = frame.shape
    mid_x, mid_y = width // 2, height // 2

    # Draw quadrants
    cv2.line(frame, (mid_x, 0), (mid_x, height), (255, 255, 255), 2)
    cv2.line(frame, (0, mid_y), (width, mid_y), (255, 255, 255), 2)

    quadrant = None  # Reset detected quadrant
    command = 4

    if ids is not None:
        ids = ids.flatten()
        for (corner, marker_id) in zip(corners, ids):
            marker_corners = corner.reshape((4, 2))
            center_x = int(marker_corners[:, 0].mean())
            center_y = int(marker_corners[:, 1].mean())

            # Determine quadrant
            if center_x >= mid_x and center_y < mid_y:
                quadrant = "00"  # Top Right
                command = 0
                color = (0, 255, 255)
            elif center_x < mid_x and center_y < mid_y:
                quadrant = "01"  # Top Left
                command = 1
                color = (255, 0, 255)
            elif center_x >= mid_x and center_y >= mid_y:
                quadrant = "10"  # Bottom Right
                command = 2
                color = (0, 255, 0)
            else:
                quadrant = "11"  # Bottom Left
                command = 3
                color = (255, 0, 0)

            text = f" Quad: {quadrant}"
            cv2.putText(frame, text, (center_x - 40, center_y - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

            cv2.circle(frame, (center_x, center_y), 10, color, -1)
            frame = aruco.drawDetectedMarkers(frame, corners)

    else:
        quadrant = "No ArUco Marker"
        command = 4
        cv2.putText(frame, quadrant, (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Only update LCD if quadrant changes (no more flashing)
    if quadrant != last_quadrant:
        q.queue.clear()  # Remove old data before adding new
        q.put(quadrant)
        i2c1.write_byte_data(ARD_ADDR, offset, command)
        last_quadrant = quadrant  # Update last displayed quadrant

    cv2.imshow("ArUco Tracker", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
