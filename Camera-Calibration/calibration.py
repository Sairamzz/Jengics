import cv2
import numpy as np
import glob
import os

# === Parameters ===
CHECKERBOARD = (8, 6)        # Number of inner corners
SQUARE_SIZE = 0.03          # Size of one square in meters (e.g., 2.5 cm)

# === Paths ===
IMAGE_DIR = "checkerboard_images"
IMAGE_PATTERN = os.path.join(IMAGE_DIR, "*.jpg")

# === Storage ===
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Prepare object points grid
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# === Loop through all images ===
images = glob.glob(IMAGE_PATTERN)
for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        print(f"[✓] Corners found in {fname}")
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display corners
        img_corners = cv2.drawChessboardCorners(img.copy(), CHECKERBOARD, corners, ret)
        cv2.imshow('Corners', img_corners)
        cv2.waitKey(200)
    else:
        print(f"[✗] Checkerboard not found in {fname}")

cv2.destroyAllWindows()

# === Calibrate the camera ===
if len(objpoints) < 5:
    print("Not enough valid images for calibration. Need at least 5.")
    exit()

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n=== Calibration Results ===")
print("Camera Matrix (K):\n", K)
print("\nDistortion Coefficients:\n", dist)
print("\nReprojection Error:", ret)

# === Save results ===
np.savez("camera_calibration_data.npz", K=K, dist=dist, rvecs=rvecs, tvecs=tvecs)
print("\n Calibration data saved to: camera_calibration_data.npz")
