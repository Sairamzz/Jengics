import cv2

from pupil_apriltags import Detector



import numpy as np

# TODO: assign actual camera intrinsic values through calibration
# Define camera intrinsic parameters (modify these for your camera)
camera_matrix = np.array( [[1.50741073e+03, 0.00000000e+00, 4.72696241e+02],
 [0.00000000e+00, 1.51427107e+03, 2.29899503e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
, dtype=np.float32)  # 0, 0, 1

# dist_coeffs = np.zeros(4)  # Assuming no lens distortion
dist_coeffs =  np.array( [[ 1.73271390e-01,  1.91603494e+00,  2.20351447e-03,  2.70285791e-02, -9.45373992e+00]])

# TODO: "DONE" use actual tags size here
tag_size = 0.095

# TODO: use actual tags size here
# Define the 3D points of the AprilTag corners in its local coordinate frame
obj_points = np.array([
    [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left
    [ tag_size / 2, -tag_size / 2, 0],  # Bottom-right
    [ tag_size / 2,  tag_size / 2, 0],  # Top-right
    [-tag_size / 2,  tag_size / 2, 0]   # Top-left
], dtype=np.float32)


detector = Detector(families='tag25h9')
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the frame
    tags = detector.detect(gray)


    for tag in tags:
        img_points = np.array(tag.corners, dtype=np.float32)

        # Estimate pose using solvePnP
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

        if success:
            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(rvec)

            print(f"Tag ID: {tag.tag_id}")
            print(f"Position (x, y, z): {tvec.ravel()}")
            print(f"Rotation Matrix:\n{R}")

            # Draw the detected tag
            for i in range(4):
                p1 = tuple(img_points[i].astype(int))
                p2 = tuple(img_points[(i + 1) % 4].astype(int))
                cv2.line(frame, p1, p2, (0, 255, 0), 2)

            # Display the center of the tag
            cX, cY = int(tag.center[0]), int(tag.center[1])
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {tag.tag_id}", (cX - 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Show the frame with detections
    cv2.imshow("AprilTag Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()