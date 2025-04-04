import cv2
from pupil_apriltags import Detector

import numpy as np
import time

# TODO: "DONE" assign actual camera intrinsic values through calibration
camera_matrix = np.array([[800.34154022,   0. ,        330.068031],
 [  0.        , 801.58117864 ,270.23508093],
 [  0.         ,  0.       ,    1.        ]], dtype=np.float32)  # 0, 0, 1


# dist_coeffs = np.zeros(4)  # Assuming no lens distortion
dist_coeffs =  np.array([[-3.27782913e-03,  1.50646740e+00,  2.47993711e-03, -7.50227307e-03, -5.82926301e+00]])

# TODO: "DONE" use actual tags size here
tag_size = 0.0096

# Define the 3D points of the AprilTag corners in its local coordinate frame
obj_points = np.array([
    [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left
    [ tag_size / 2, -tag_size / 2, 0],  # Bottom-right
    [ tag_size / 2,  tag_size / 2, 0],  # Top-right
    [-tag_size / 2,  tag_size / 2, 0]   # Top-left
], dtype=np.float32)

# CAM_DISTANCE_FROM_ARM = 0.06 # meters


def get_tag_coords(arm_control):
    detector = Detector(families='tag25h9')
    cap = cv2.VideoCapture(2)

    while not cv2.waitKey(1) & 0xFF == ord('q'):
        _, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = detector.detect(gray)


        for tag in tags:
            if tag.tag_id is not 14:
                continue

            img_points = np.array(tag.corners, dtype=np.float32)

            # Estimate pose using solvePnP
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

            if success:
                for i in range(4):
                    p1 = tuple(img_points[i].astype(int))
                    p2 = tuple(img_points[(i + 1) % 4].astype(int))
                    cv2.line(frame, p1, p2, (0, 255, 0), 2)                
                    # tvec.ravel()
                    
                x, y, z = tvec.ravel()

                # xr, yr, zr = z + 0.1, -x, -y
                xr = z
                yr = x
                zr = -y

                if xr < 0.4: 
                    xr = 0.4
                if zr < 0.08:
                    zr = 0.08
                # bot.arm.set_ee_pose_components(x=0.4, y=yr, z=zr, roll=0.0, pitch=0.0, yaw=0.0)
                
                arm_control.execute_target(xyz=[xr, yr, zr], 
                    quaternion=[0.0, 0.0, 0.0, 1.0]
                )
                # arm_control.move_gripper(gripper_val, -gripper_val)


        # Show the frame with detections
        cv2.imshow("AprilTag Detection", frame)
