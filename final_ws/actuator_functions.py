# push out piece
def push(bot, goal_x, goal_y, goal_z):
    push_dist = 0.05
    bot.arm.set_ee_pose_components(x=goal_x + push_dist, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)


# pick pushed out piece
def pick(bot, goal_x, goal_y, goal_z):
    pick_dist = 0.03
    bot.gripper.release(1)
    bot.arm.set_ee_pose_components(x=goal_x + pick_dist, y=goal_y, z=goal_z, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.gripper.grasp(1)
    bot.gripper.set_pressure(0.8)


# place on top picked piece
def place(bot, goal_x, goal_y, goal_z):
    place_dist = 0.1
    height_gain = 0.1
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.arm.set_ee_pose_components(x=goal_x + place_dist, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)

    bot.gripper.release(2)
    bot.arm.set_ee_pose_components(x=goal_x, y=goal_y, z=goal_z + height_gain, roll=0.0, pitch=0.0, yaw=0.0, blocking=True)
    bot.gripper.grasp()