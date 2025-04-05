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

    #loop
    x, y, z = get_tag_coords(bot)

    robot_shutdown()


if __name__ == '__main__':
    main()
