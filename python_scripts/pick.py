#!/usr/bin/env python3
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import time

def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()
    
    # bot.arm.set_ee_pose_components(x = 0.2, y=0.0, z = 0.05, roll=3.14)
    
    # bot.arm.go_to_home_pose(moving_time=3)

    bot.arm.set_ee_pose_components(x=0.3, z=0.3, moving_time=1)
    # bot.arm.set_ee_pose_components(x=0.3, z=0.3, roll=1.57, moving_time=1)
    # bot.arm.set_ee_pose_components(x=0.3, z=0.3, roll=-1.57, moving_time=2)


    # bot.arm.set_ee_pose_components(x=0.3, z=0.3, moving_time=1)
    # bot.arm.set_ee_pose_components(x=0.3, z=0.3, pitch=1.57, moving_time=1)
    bot.arm.set_ee_pose_components(x=0.2, z=0.5, roll=3.14, pitch=-1.57, moving_time=2)


    # bot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.3, moving_time=0.5)

    # bot.arm.set_ee_pose_components(x=0.2,y=-0.1, z=0.3, moving_time=0.5)


    # bot.arm.set_ee_pose_components(x=0.2,y=-0.1, z=0.6, roll=1.57, moving_time=3)

    # bot.arm.set_ee_pose_components(x=0.3, z=0.4, roll=3.14, moving_time=5)

    bot.arm.go_to_sleep_pose()
    

    robot_shutdown()


if __name__ == '__main__':
    main()

# bot.gripper.grasp(2.0)
    # bot.gripper.set_pressure(1.0)
    
    # bot.gripper.release(2.0)

    # bot.arm.go_to_sleep_pose()