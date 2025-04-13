import time
import numpy as np
# push out piece
def push(bot, goal_x, goal_y, goal_z, tag_id):
    push_dist = 0.09
    bot.arm.set_ee_pose_components(x=goal_x + push_dist, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0)
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0)
    bot.arm.set_ee_pose_components(x=goal_x - 0.02, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0)
    

    # left = [27, 21, 15, 9, 3, 24, 18, 12, 6, 0]
    # right = [29, 23, 17, 11, 5, 26, 20, 14, 8, 2]


    # if tag_id in left:
    #     bot.arm.set_ee_pose_components(x=goal_x, y=goal_y - 0.05, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0)

    # elif tag_id in right:
    #     bot.arm.set_ee_pose_components(x=goal_x, y=goal_y + 0.05, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0)
    
    # else:
    #     bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0)



# pick pushed out piece
def pick(bot, goal_x, goal_y, goal_z):
    pick_dist = 0.04
    bot.gripper.release(1)
    bot.arm.set_ee_pose_components(x=goal_x + pick_dist, y=goal_y + 0.01, z=goal_z, roll=np.pi/2, pitch=0.0, yaw=0.0, blocking=True)
    bot.gripper.grasp(1)
    bot.gripper.set_pressure(1.0)

    bot.arm.set_ee_pose_components(x=goal_x - 0.06, y=goal_y, z=goal_z, roll=np.pi/2, pitch=0.0, yaw=0.0, blocking=True)

    # bot.arm.set_ee_pose_components(x=goal_x - 0.06, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)

    


# place on top picked piece
def place(bot, goal_x, goal_y, goal_z):
    place_dist = 0.1
    height_gain = 0.13
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z + height_gain, roll=np.pi/2, pitch=0.0, yaw=0.0, blocking=True)
    bot.arm.set_ee_pose_components(x=goal_x + place_dist, y=goal_y, z=goal_z + height_gain, roll=np.pi/2, pitch=0.0, yaw=0.0, blocking=True)

    bot.gripper.release(2)
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.gripper.grasp()