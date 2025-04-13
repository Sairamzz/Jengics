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

    selected_tag_id = 16


    pushed = False

    goal_x = None
    goal_y = None
    goal_z = None

    while not cv2.waitKey(1) & 0xFF == ord('q'):
        # print(pushed)
        _, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = detector.detect(gray)

        now = time.time()
        
        key = cv2.waitKey(1) & 0xFF
        

        if key in range(ord('0'), ord('9') + 1):
            
            selected_tag_id = int(chr(key))
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('a'):
            selected_tag_id = 10
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('s'):
            selected_tag_id = 11
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        
        if key == ord('d'):
            selected_tag_id = 12
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        
        if key == ord('f'):
            selected_tag_id = 13
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('g'):
            selected_tag_id = 14
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('h'):
            selected_tag_id = 15
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('j'):
            selected_tag_id = 16
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('k'):
            selected_tag_id = 17
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('l'):
            selected_tag_id = 18
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('z'):
            selected_tag_id = 19
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('x'):
            selected_tag_id = 20
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('c'):
            selected_tag_id = 21
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('v'):
            selected_tag_id = 22
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('b'):
            selected_tag_id = 23
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('n'):
            selected_tag_id = 24
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('m'):
            selected_tag_id = 25
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord(','):
            selected_tag_id = 26
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('.'):
            selected_tag_id = 27
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('/'):
            selected_tag_id = 28
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False

        if key == ord('q'):
            selected_tag_id = 29
            print(f"Selected tag ID: {selected_tag_id}")
            pushed = False



        for tag in tags:
            if tag.tag_id != selected_tag_id:
                continue

            img_points = np.array(tag.corners, dtype=np.float32)

            # Estimate pose using solvePnP
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

            if now - last_exec_time > interval and not pushed:
                if success:
                    # Display the center of the tag
                    cX, cY = int(tag.center[0]), int(tag.center[1])
                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"ID: {tag.tag_id}", (cX - 10, cY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


                    x, y, z = tvec.ravel()


                    goal_x = 0.35
                    #z + 0.1
                    goal_y = x - 0.028
                    goal_z = -y + 0.137 - 0.025

                    if goal_x < 0.3: 
                        goal_x = 0.3
                    if goal_z < 0.08:
                        goal_z = 0.08

                    # x_current, y_current, z_current = bot.arm.get_ee_pose()

                    print("x ", goal_x)
                    print("y ", goal_y)
                    print("z ", goal_z)
                    
                    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=False)
                    last_exec_time = now

        if key == ord('p') and goal_x:
            pushed = True
            push(bot, goal_x, goal_y, goal_z, selected_tag_id)
            print(f"Pushing: {selected_tag_id}")
            
        
        if key == ord('o') and goal_x:
            pick(bot, goal_x, goal_y, goal_z)
            pushed = True

        if key == ord('i') and goal_x:
            place(bot, goal_x, goal_y, goal_z)
        
        if key == ord('s'):
            bot.arm.go_to_sleep_pose()



        # Show the frame with detections
        cv2.imshow("AprilTag Detection", frame)
