#!/usr/bin/env python3
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import numpy as np
import time

from detect_april_tags import get_tag_coords



def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()
    bot.gripper.grasp()

    x, y, z = get_tag_coords(bot)
    
    

    # go up
    # bot.arm.set_ee_pose_components(x=0.4, y=0.0, z=0.1)
    #bot.arm.set_ee_pose_components(x=0.4, y=0.0, z=0.3)

    # bot.arm.go_to_sleep_pose(moving_time=3)


    # bot.gripper.grasp()
    # bot.gripper.release()
    # bot.arm.set_ee_pose_components(x=0.4, y=0.0, z=0.1)

    # bot.arm.go_to_sleep_pose(moving_time=3)
    
    robot_shutdown()


if __name__ == '__main__':
    main()

    # # push out
    # bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.078, moving_time=5)
    # bot.arm.set_ee_pose_components(x=0.35, y=0.0, z=0.078, moving_time=5)
    # bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.078, moving_time=3)

    # # safe spot to go up
    # bot.arm.go_to_sleep_pose(3)
    # bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.35)
    # bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.pi/2.0)
    # bot.gripper.release()
    # bot.arm.set_ee_pose_components(x=0.4)
