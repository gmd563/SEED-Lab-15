import cv2
import numpy as np

def adjust_intensity(frame, alpha=1.0, beta=0):
    adjusted = np.clip(alpha * frame + beta, 0, 255).astype(np.uint8)
    return adjusted

# Open the default camera (use 0 for the built-in camera, or another index for external)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

alpha = 1.5  # Contrast control
beta = 20    # Brightness control

while True:
    ret, frame = cap.read()
    if not ret:
        break

    adjusted_frame = adjust_intensity(frame, alpha, beta)

    cv2.imshow('Adjusted Video', adjusted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()

