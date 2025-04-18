import cv2
import numpy as np
from cv2 import aruco

# Initialize camera
camera = cv2.VideoCapture(0)

# Load Aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

# Load camera calibration data
calibration_data = np.load('camera_calibration_data.npz')
camera_matrix = calibration_data['camera_matrix'] # mtx
dist_coeffs = calibration_data['dist_coeffs'] # coeffs

# Set Aruco marker size
marker_size = 0.05  # in meters

# camera HFOV
camera_fov = 61

# Define HSV color ranges
lower_green = np.array([40, 100, 50])
upper_green = np.array([80, 255, 255])
lower_red1 = np.array([0, 150, 80])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 150, 80])
upper_red2 = np.array([180, 255, 255])

# Intensity calibration variables
alpha = 1.5  # Contrast control
beta = 20    # Brightness control

def adjust_intensity(frame, alpha=1.0, beta=0):
    adjusted = np.clip(alpha * frame + beta, 0, 255).astype(np.uint8)
    return adjusted

# arrow detection
def detect_arrow_color(mask, draw_on):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
            if len(approx) >= 5:
                arrow_like_contours.append((area, contour))
        
        if arrow_like_contours:
        arrow_like_contours.sort(reverse=True)  # Largest contour first
        cv2.drawContours(draw_on, [contour], -1, (255, 0, 0), 2)
        return True  # Arrow of this color detected
    return False

while True:
    ret, frame = camera.read()
    if not ret:
        print("Camera failure.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    selected_corners = None
    selected_id = None

    if ids is not None:
        ids = ids.flatten()
        min_distance = float('inf')
        height, width, _ = frame.shape
        mid_x = width // 2

        for (corner, marker_id) in zip(corners, ids):
            marker_corners = corner.reshape((4, 2)).astype(np.int32)
            marker_width_px = np.linalg.norm(marker_corners[0] - marker_corners[1])
            marker_height_px = np.linalg.norm(marker_corners[1] - marker_corners[2])
            marker_size_px = (marker_width_px + marker_height_px) / 2
            focal_length = camera_matrix[0, 0]
            distance = ((marker_size * focal_length) / marker_size_px) * 3.28084

            if distance < min_distance:
                min_distance = distance
                selected_id = marker_id
                selected_corners = marker_corners

        if selected_corners is not None:
            rect = cv2.minAreaRect(selected_corners)
            (cx, cy), (w, h), angle = rect
            if w < h:
                w, h = h, w
            extend_w = int(1.5 * w)

            box = cv2.boxPoints(((cx, cy), (extend_w, int(h)), angle))
            destination_pts = np.array([[0,0], [extend_w - 1, 0], [extend_w - 1, h - 1], [0, h - 1]], dtype="float32")
            perspective_mtx = cv2.getPerspectiveTransform(box.astype("float32"), destination_pts)
            crop_hsv = cv2.warpPerspective(hsv, perspective_mtx, (extend_w, int(h)))
            crop_bgr = cv2.warpPerspective(frame, perspective_mtx, (extend_w, int(h)))  # for display with contours

            # Color masks
            mask_green = cv2.inRange(crop_hsv, lower_green, upper_green)
            mask_red1 = cv2.inRange(crop_hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(crop_hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, k)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, k)

            # Detect and draw arrows
            red_arrow = detect_arrow_color(mask_red, crop_bgr)
            green_arrow = detect_arrow_color(mask_green, crop_bgr)

            if red_arrow:
                cv2.putText(crop_bgr, "Red Arrow Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            if green_arrow:
                cv2.putText(crop_bgr, "Green Arrow Detected", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            cv2.imshow("Cropped Arrow View", crop_bgr)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
