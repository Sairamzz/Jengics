import cv2
import numpy as np

# Checkerboard (inner corners, NOT squares)
CHECKERBOARD = (8, 6)  # example: 10x7 squares → 9x6 corners

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

print("📷 Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Try to detect corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, flags)

    if ret:
        frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)
        cv2.putText(frame, "✓ Checkerboard Detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "✗ Not Detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Checkerboard Detection", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
