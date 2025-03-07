# Ryan Cockrill
# Takes pic and saves it as jpg
import cv2
import os

# Create a directory for captured images if it doesn't exist
save_dir = "captured_images"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Open the camera
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

image_count = 1

while True:
    # Capture frame from the camera
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Show the captured frame
    cv2.imshow("Press 's' to save, 'q' to quit", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        # Find the next available image number
        while os.path.exists(os.path.join(save_dir, f"img{image_count}.jpg")):
            image_count += 1
        
        # Save the image
        image_path = os.path.join(save_dir, f"img{image_count}.jpg")
        cv2.imwrite(image_path, frame)
        print(f"Image saved as {image_path}")

    elif key == ord("q"):
        print("Exiting without saving.")
        break

# Release the camera and close windows
camera.release()
cv2.destroyAllWindows()
