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

    # bot.arm.go_to_home_pose(moving_time=2)
    

    bot.arm.go_to_sleep_pose()
    bot.gripper.grasp()

    # print(bot.arm.get_ee_pose())

    #loop
    

    x, y, z = get_tag_coords(bot)
    # bot.arm.set_ee_pose_components(x=0.3 , y=0.0, z=0.2, roll=np.pi/2, pitch=0.0, yaw=0.0, blocking=True)

    # bot.arm.set_ee_pose_components(x=0.4 , y=0.0, z=0.2, roll=0.0, pitch=3.14 / 2, yaw=0.0, blocking=True)
    # bot.arm.set_ee_pose_components(x=0.3 , y=0.0, z=0.2, roll=0.0, pitch=3.14 / 2, yaw=0.0, blocking=True)
    

    robot_shutdown()


if __name__ == '__main__':
    main()
