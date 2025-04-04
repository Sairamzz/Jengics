import cv2
from pupil_apriltags import Detector

import numpy as np
import time

# # TODO: "DONE" assign actual camera intrinsic values through calibration
# camera_matrix = np.array([[800.34154022,   0. ,        330.068031],
#  [  0.        , 801.58117864 ,270.23508093],
#  [  0.         ,  0.       ,    1.        ]], dtype=np.float32)  # 0, 0, 1


# # dist_coeffs = np.zeros(4)  # Assuming no lens distortion
# dist_coeffs =  np.array([[-3.27782913e-03,  1.50646740e+00,  2.47993711e-03, -7.50227307e-03, -5.82926301e+00]])


camera_matrix = np.array( [[526.72911527, 0.0, 320.98251076],
 [0.0, 528.41016751, 251.78354461],
 [0.0, 0.0, 1.0]]
, dtype=np.float32)  # 0, 0, 1

dist_coeffs =  np.array( [[ 0.0325677 , -0.10725443 , 0.0055427  , 0.01178159 , 0.9617012 ]])



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

def push(bot, goal_x, goal_y, goal_z):
    push_dist = 0.05
    bot.arm.set_ee_pose_components(x=goal_x + push_dist, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)


def pick(bot, goal_x, goal_y, goal_z):
    pick_dist = 0.03
    bot.gripper.release(1)
    bot.arm.set_ee_pose_components(x=goal_x + pick_dist, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.gripper.grasp(1)
    bot.gripper.set_pressure(0.8)


def place(bot, goal_x, goal_y, goal_z):
    place_dist = 0.1
    height_gain = 0.1
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.arm.set_ee_pose_components(x=goal_x + place_dist, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)

    bot.gripper.release(2)
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.gripper.grasp()



def get_tag_coords(bot):
    detector = Detector(families='tag25h9')
    cap = cv2.VideoCapture(0)

    interval = 1  # seconds
    last_exec_time = time.time() - interval

    selected_tag_id = 27

    while not cv2.waitKey(1) & 0xFF == ord('q'):
        _, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = detector.detect(gray)

        now = time.time()
        
        key = cv2.waitKey(1) & 0xFF
        

        if key in range(ord('0'), ord('9') + 1):
            selected_tag_id = int(chr(key))
            print(f"Selected tag ID: {selected_tag_id}")


        for tag in tags:
            if tag.tag_id != selected_tag_id:
                continue

            img_points = np.array(tag.corners, dtype=np.float32)

            # Estimate pose using solvePnP
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

            if success:
                # Display the center of the tag
                cX, cY = int(tag.center[0]), int(tag.center[1])
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {tag.tag_id}", (cX - 10, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


                x, y, z = tvec.ravel()


                goal_x = z + 0.13
                goal_y = x
                goal_z = -y + 0.13

                if goal_x < 0.35: 
                    goal_x = 0.35
                if goal_z < 0.08:
                    goal_z = 0.08

                # x_current, y_current, z_current = bot.arm.get_ee_pose()

                print("x ", goal_x)
                print("y ", goal_y)
                print("z ", goal_z)
                if now - last_exec_time > interval:
                    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=False)
                    last_exec_time = now

                if key == ord('p'):
                    push(bot, goal_x, goal_y, goal_z)
                    print(f"Pushing: {selected_tag_id}")
                
                if key == ord('o'):
                    pick(bot, goal_x, goal_y, goal_z)
                    print(f"Pushing: {selected_tag_id}")

                if key == ord('i'):
                    place(bot, goal_x, goal_y, goal_z)
                    print(f"Pushing: {selected_tag_id}")


        # Show the frame with detections
        cv2.imshow("AprilTag Detection", frame)
