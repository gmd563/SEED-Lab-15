#red and green detection testing

import cv2
import numpy as np

def detect_markers():
    # Open the camera
    camera = cv2.VideoCapture(0)
    
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return
    
    while True:
        # Capture frame
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define green color range
        lower_green = np.array([40, 100, 50])
        upper_green = np.array([80, 255, 255])

        # Define red color range
        lower_red1 = np.array([0, 150, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 80])
        upper_red2 = np.array([180, 255, 255])

        # Create masks
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)  # Combine red masks

        #(reduce noise, keep arrow shapes)
        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Minimum area to ignore noise but still detect markers/arrows
        min_area = 250  

        def filter_contours(contours, color):
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_area:
                    # Approximate shape to filter out other objects
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

                    # If shape has 5-7 vertices, it may be an arrowhead
                    if 5 <= len(approx) <= 7:
                        cv2.drawContours(frame, [approx], -1, color, 3)  # Draw detected marker

        # Process green and red
        filter_contours(contours_green, (0, 255, 0))  # Green
        filter_contours(contours_red, (0, 0, 255))  # Red

        # Show frame
        cv2.imshow("Detected Markers", frame)

        # Exit condition
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_markers()
