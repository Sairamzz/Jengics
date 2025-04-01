import cv2
import os

# === Parameters ===
SAVE_DIR = "checkerboard_images"
CAMERA_ID = 0  # Default webcam

# Create folder if it doesn't exist
os.makedirs(SAVE_DIR, exist_ok=True)

# Start camera
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("Press 'c' to capture, 'q' to quit.")

img_counter = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Live Feed - Checkerboard Capture", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        filename = f"{SAVE_DIR}/img_{img_counter:02d}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        img_counter += 1

    elif key == ord('q'):
        print("👋 Quitting...")
        break

cap.release()
cv2.destroyAllWindows()
