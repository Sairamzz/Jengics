import cv2
import numpy as np

# Load calibration data
data = np.load("camera_calibration_data.npz")
K = data["K"]
dist = data["dist"]

# Load test image
img = cv2.imread("checkerboard_images/img1.jpg")
undistorted = cv2.undistort(img, K, dist)

cv2.imshow("Original", img)
cv2.imshow("Undistorted", undistorted)
cv2.waitKey(0)
cv2.destroyAllWindows()
