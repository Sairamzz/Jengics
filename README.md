# Jengics

This project was done as a part of the CS 5335 (Robotics Science and Systems) course at Northeastern University.

* [Objectives](#Objectives)
* [Features](#Features)
* [Implementaion](#Implementaion)
* [Results](#Results)
   * [Final Working Video](#FinalWorkingVideo)
* [Contributors](#Contributors)
* [RobotArm](#RobotArm)

## Objectives:
The goal of this project was to develop a robotic system capable of playing Jenga by autonomously identifying, selecting, extracting, and repositioning blocks without causing the tower to collapse. Unlike traditional pick-and-place tasks, Jenga presents unique challenges in precision, force control, and adaptability under real-world physical constraints.

The objective was to design and integrate a complete system—perception, planning, and manipulation—using an Interbotix WidowX 250s robot arm, custom hardware, and vision-based detection methods, in order to demonstrate the feasibility of fine robotic manipulation in constrained environments.

## Features:
This robotic system combines real-time vision, motion planning, and manipulation strategies to tackle the complexities of Jenga. 

- Each block in the tower was labeled with a unique AprilTag, which enabled reliable block detection and pose estimation using a Logitech C270 webcam and OpenCV’s solvePnP function.
- The system employed ROS2 and MoveIt for inverse kinematics, ensuring precise control of the WidowX 250s arm and its custom 3D-printed gripper.
- To extend workspace accessibility, a manually operated rotating base was introduced, allowing the robot to interact with blocks on all sides of the tower.
- Additional features included modular ROS2 controllers for the arm and gripper, as well as text-to-speech prompts for human-robot interaction during the manipulation sequence.

## Implementaion:

- The system was built around the Interbotix WidowX 250s robotic arm, controlled via ROS2 and MoveIt for trajectory execution.
- Perception was handled by a webcam-based vision pipeline that identified AprilTags on the blocks, estimated their 3D poses, and transformed them into the robot’s base frame for accurate positioning.

The manipulation strategy followed a three-step process: push, pick, and place.
- The robot first pushed the block outward, then after the tower was rotated on the base, it grasped the block and placed it on top of the stack. ROS2 nodes were developed for arm and gripper control, enabling joint trajectory publishing and real-time actuation.

A custom gripper was iteratively designed and refined to allow precise interaction with the narrow gaps between Jenga blocks. User interaction during demonstrations was facilitated by keyboard commands that triggered each manipulation phase, alongside auditory prompts from the robot to guide tower rotation.

<img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/73bc7823-99c3-4cf3-8eb9-303ab830ca74" />

(Environment Setup)

<img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/8c20bb12-5bf1-46d1-88d3-8d61c0a155c1" />

(Jenga Tower)

## Results:

The Jenga-playing robot successfully demonstrated the ability to extract and reposition blocks from the tower under real-world constraints. Experiments showed reliable block detection, stable manipulation, and repeatable task execution, validating the integration of perception, planning, and control modules. Despite limitations such as the restricted reach of the WidowX 250s, reliance on manual rotation of the tower, and the low resolution of the webcam, the system achieved its primary objective. 

## We Initially Hard-coded to conceptualize how it should work

https://github.com/user-attachments/assets/16d30041-4d4a-48c0-81ad-9393f4f4ff36

## Final Working Video

https://github.com/user-attachments/assets/e708a22d-34af-4f8b-afca-50bff3af8164

## Contributors
- [Sairam Sridharan](https://github.com/Sairamzz)
- [Rituraj Navindgikar](https://github.com/rituraj-navindgikar)
- Reem Almazroa
- Alay Shah

## RobotArm:

If you'd like to run the trossen robotics' bot refer these websites:

[Arm Description](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_descriptions.html)

[Arm Control](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_control.html)

[Simulation Configuration](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/simulation_configuration.html)

[Move It Motion Planning](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/moveit_motion_planning_configuration.html)



