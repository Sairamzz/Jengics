#!/usr/bin/env python3
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import numpy as np
import time


from scipy.spatial.transform import Rotation as R

def compute_rpy_from_xyz(x, y, z):
    # Assume Z-axis is pointing in the direction of (x, y, z)
    z_axis = np.array([x, y, z])
    z_axis = z_axis / np.linalg.norm(z_axis)  # Normalize
    
    # Assume X-axis is fixed (you may need to adjust based on your setup)
    x_axis = np.array([1, 0, 0])
    
    # Compute Y-axis as the cross product
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # Compute corrected X-axis
    x_axis = np.cross(y_axis, z_axis)
    
    # Construct rotation matrix
    R_mat = np.vstack([x_axis, y_axis, z_axis]).T
    
    # Convert to roll-pitch-yaw
    rpy = R.from_matrix(R_mat).as_euler('xyz', degrees=False)
    
    return rpy[0], rpy[1], rpy[2]

# Example XYZ position
rpy_angles = compute_rpy_from_xyz(0.2, 0.1, 0.2)


def move (bot, x, y, z, time):
    roll, pitch, yaw = compute_rpy_from_xyz(x, y, z)
    
    bot.arm.set_ee_pose_components(x, y, z, roll, pitch, yaw, moving_time=time)


def matrix_move(bot, matrix, time):
    bot.arm.set_ee_pose_matrix(matrix, moving_time=time)


def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()
    bot.gripper.grasp()

    # def movin():
#         matrix_move(bot, [[ 0.98075174,  0.15827041,  0.11435253,  0.26298288],
#     [-0.16433774,  0.98534593,  0.04567824, -0.03348954],
#     [-0.10544729, -0.06359145,  0.99238954,  0.06986783],
#     [ 0.,          0.,          0.,          1.        ]], 5)


#     gripper close

#     move to jenga 
#         matrix_move(bot, [[ 0.9992143,   0.01652461,  0.03602392,  0.28775867],
#  [-0.01540961 , 0.9994002 , -0.03101262 , 0.01129435],
#  [-0.03651479 , 0.03043314,  0.99886961 , 0.06806993],
#  [ 0.         , 0.        ,  0.         , 1.        ]], 5)

#         bot.arm.set_ee_cartesian_trajectory(x=0.03)
#     align with jenga
#         matrix_move(bot, [[ 0.99801509,  0.05162404, -0.03606704,  0.31838851],
#  [-0.05152127  ,0.99866477,  0.00377362 , 0.01508095],
#  [ 0.03621369 ,-0.00190791  ,0.99934225 , 0.052901  ],
#  [ 0.     ,     0.   ,       0.  ,        1.        ]], 5)


#     # push

#         matrix_move(bot, [[ 0.99291388,  0.06846027, -0.09713504,  0.3572802 ],
#  [-0.06958418,  0.99754216 ,-0.00822658 , 0.01610571],
#  [ 0.09633311  ,0.01492734,  0.99523721 , 0.05587014],
#  [ 0.         , 0.  ,        0.  ,        1.        ]] , 5)

# # intermiediate

#         matrix_move(bot, [[ 0.99766196,  0.05143713, -0.04499807,  0.31055844],
#  [-0.05066894 , 0.99855239 , 0.01804966 , 0.01123098],
#  [ 0.04586135 ,-0.01572745 , 0.998824   , 0.05350855],
#  [ 0.          ,0.,          0.         , 1.        ]], 5)


#     # come to "save pos 1"

#         matrix_move(bot, [[ 0.99971471,  0.00349894, -0.0236274,   0.1936003 ],
#  [-0.00407579 , 0.99969371 ,-0.02441069 , 0.00384316],
#  [ 0.02353475 , 0.02450003  ,0.99942277,  0.1209038 ],
#  [ 0.         , 0.   ,       0.        ,  1.        ]] , 5)


#     # go up 

#         matrix_move(bot, [[ 0.99836554,  0.04295165, -0.03770161,  0.17640855],
#  [-0.0405254  , 0.99719569 , 0.06291614 ,-0.00670388],
#  [ 0.04029823 ,-0.06128544 , 0.99730645 , 0.45393654],
#  [ 0.          ,0.         , 0.         , 1.        ]], 5)

#     # gripper open

#     # go straight front

#         matrix_move(bot, [[ 0.83819468, -0.03505329,  0.54424346,  0.53717312],
#  [ 0.00906963,  0.99869021,  0.05035478 , 0.00267846],
#  [-0.54529572, -0.03727102,  0.83741474 , 0.27433585],
#  [ 0.        ,  0.        ,  0.         , 1.        ]], 5)


#     come down to pick


# [[ 0.06845977 -0.07681219  0.99469249  0.48178738]
#  [-0.0337193   0.99628386  0.07925581  0.01199839]
#  [-0.99708388 -0.03896617  0.06561531  0.13286213]
#  [ 0.          0.          0.          1.        ]]


    # gripper close

    # pull out far enough

    # go "save pos 2"

    # place on top
    
    # gripper open

    # gripper close

    # home - bye bye

    # break tower


    # movin()
    


    # while True:
    #     print(bot.arm.get_ee_pose())
    #     print()
    #     time.sleep(1)   


    x, y, z = 0.1, 0.0, 0.5

    # waist, shoulder, elbow,forearm roll, wrist angle, wrist rotate, gripper
    # bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.pi/2.0)
    # bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi/2.0)
    
    # move(bot, x, y, z, 5)
    bot.gripper.grasp()

    bot.arm.go_to_sleep_pose(3)
    
    # push out
    bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.078, moving_time=5)
    bot.arm.set_ee_pose_components(x=0.35, y=0.0, z=0.078, moving_time=5)
    bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.078, moving_time=3)

    # safe spot to go up
    bot.arm.go_to_sleep_pose(3)
    bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.35)
    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.pi/2.0)
    bot.gripper.release()
    bot.arm.set_ee_pose_components(x=0.4)

    # go up
    #bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.1)
    #bot.arm.set_ee_pose_components(x=0.4, y=0.0, z=0.3)

    # go down for picking
    # pose_components(x=0.5, y=0.0, z=0.2)

    #bot.gripper.release()
    

    bot.arm.go_to_sleep_pose(moving_time=3)
    robot_shutdown()


if __name__ == '__main__':
    main()

# LEVEL 3 position
# bot.arm.set_ee_pose_components(x=0.25, z=0.05) 