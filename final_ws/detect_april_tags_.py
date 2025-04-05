import cv2
from pupil_apriltags import Detector

import numpy as np
import time

from actuator_functions import push, pick, place

from camera_intrinsic import camera_matrix, dist_coeffs, obj_points, CAMERA_MODE



def get_tag_coords(bot):
    detector = Detector(families='tag25h9')
    cap = cv2.VideoCapture(CAMERA_MODE)

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
                    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y + 0.0125, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=False)
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
