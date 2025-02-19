# Ryan Cockrill
# Mini Project
# Recognizes markers in one of 4 quadrants from camera

#Imports
import cv2
import numpy as np
from cv2 import aruco

# Initialize camera
camera = cv2.VideoCapture(0)

# Load ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Get frame dimensions
    height, width, _ = frame.shape
    mid_x, mid_y = width // 2, height // 2

    # Draw quadrants on the screen
    cv2.line(frame, (mid_x, 0), (mid_x, height), (255, 255, 255), 2)
    cv2.line(frame, (0, mid_y), (width, mid_y), (255, 255, 255), 2)

    #code from assignment 2.2a
    if ids is not None:
        ids = ids.flatten()
        for (corner, marker_id) in zip(corners, ids):
            marker_corners = corner.reshape((4, 2))
            center_x = int(marker_corners[:, 0].mean())
            center_y = int(marker_corners[:, 1].mean())

            # Determine quadrants (Binary Output)
            if center_x >= mid_x and center_y < mid_y:
                quadrant = "00"  # Top Right
                color = (0, 255, 255)  # Yellow
            elif center_x < mid_x and center_y < mid_y:
                quadrant = "01"  # Top Left
                color = (255, 0, 255)  # Purple
            elif center_x >= mid_x and center_y >= mid_y:
                quadrant = "10"  # Bottom Right
                color = (0, 255, 0)  # Green
            else:
                quadrant = "11"  # Bottom Left
                color = (255, 0, 0)  # Blue

            # Display marker info on screen
            #text = f"ID: {marker_id}, Q: {quadrant}"
            text = f" Quad: {quadrant}"
            cv2.putText(frame, text, (center_x - 40, center_y - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

            # Highlight detected marker
            cv2.circle(frame, (center_x, center_y), 10, color, -1)
            frame = aruco.drawDetectedMarkers(frame, corners)

    else:
        # If no marker is detected, display message
        cv2.putText(frame, "No ArUco Marker Found", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Show frame
    cv2.imshow("ArUco Tracker", frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()
