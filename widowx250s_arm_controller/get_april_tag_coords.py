import cv2
from pupil_apriltags import Detector
import numpy as np

# TODO: "DONE" assign actual camera intrinsic values through calibration
camera_matrix = np.array([[526.86714797,   0. ,        323.39162405],
 [  0.        , 529.07894094 ,248.38204922],
 [  0.         ,  0.       ,    1.        ]], dtype=np.float32)  # 0, 0, 1

# dist_coeffs = np.zeros(4)  # Assuming no lens distortion
dist_coeffs =  np.array([[-1.36932426e-01,  1.77715703e+00,  2.62413066e-03,  5.40483909e-03, -4.81047475e+00]])

# Define the real-world size of the AprilTag (modify according to your tag size in meters)
tag_size = 0.01

# TODO: use actual tags size here
# Define the 3D points of the AprilTag corners in its local coordinate frame
obj_points = np.array([
    [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left
    [ tag_size / 2, -tag_size / 2, 0],  # Bottom-right
    [ tag_size / 2,  tag_size / 2, 0],  # Top-right
    [-tag_size / 2,  tag_size / 2, 0]   # Top-left
], dtype=np.float32)

# CAM_DISTANCE_FROM_ARM = 0.06 # meters

detector = Detector(families='tag25h9')
cap = cv2.VideoCapture(0)

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

            # print(f"Tag ID: {tag.tag_id}")
            # print(f"Position (x, y, z): {tvec.ravel()}")
            # print(f"Rotation Matrix:\n{R}")

            x, y, z = tvec.ravel()
            xr, yr, zr = z, -x, -y

            print()
            print("x" + str(x))
            print("y" + str(y))
            print("z" + str(z))
            print()
            

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




