# Ryan Cockrill
# Camera Calibration data .npz
# Takes images from calibration_images and calibrates the camera

import numpy as np
import cv2
import glob

# Define the dimensions of the chessboard (number of inner corners per row and column)
chessboard_size = (7, 7)  # Change this to match your chessboard
frame_size = (1280, 720)  # Resolution of the camera

# Prepare object points (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Load images for calibration
images = glob.glob('calibration_images/*.jpg')  # Ensure the path is correct

if not images:
    print("Error: No images found in the 'calibration_images' folder.")
    exit()

for fname in images:
    img = cv2.imread(fname)
    
    if img is None:
        print(f"Error: Unable to load image {fname}. Check the file format.")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        # added in
        corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

        objpoints.append(objp)
        # changed corners to corners_refined
        imgpoints.append(corners_refined)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboard_size, corners_refined, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(500)
    else:
        print(f"Chessboard corners not found in image: {fname}")

cv2.destroyAllWindows()

# Check if any valid images were found
if not objpoints:
    print("Error: No valid images with chessboard corners found.")
    exit()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_size, None, None)

#h, w = img.shape[:2]

# added in - Optimized camera matrix
#camera_matrix, roi = cv2.getOptimalNewCameraMatrix(old_matrix, dist_coeffs, (w,h), 1, (w,h))

# added in - undistort
#dst = cv2.undistort(img, old_matrix, dist_coeffs, None, camera_matrix)

# added in - crop image
#x,y,w,h = roi
#dst = dst[y:y+h, x:x+w]
#cv2.imwrite('calibImg.jpg',dst)

if not ret:
    print("Error: Calibration failed. Check the input data.")
    exit()

# Save the calibration data to a file
np.savez('camera_calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

print("Calibration successful!")
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)

total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

print("\nReprojection Error:", total_error / len(objpoints))

