from arm_control import IKClient, Wx250sArmController, Wx250sGripperController
from arm_control import compute_rpy_from_xyz

import numpy as np
import tf_transformations as tf

import time

import cv2
import rclpy

from detect_april_tags import get_tag_coords


class WidowX250s(Wx250sArmController, Wx250sGripperController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Wait for joint states to be available
        while self.current_positions is None or len(self.current_positions) == 0:
            rclpy.spin_once(self)
            print("Waiting for valid joint state...")

    def execute_target(self, xyz, quaternion=[0.0, 0.0, 0.0, 1.0], movement_time=2, velocity=[0.5] * 6, acceleration=[0.1] * 6, gripper_val=None):
        velocity = velocity or [0.5] * 6
        acceleration = acceleration or [0.1] * 6

        IK = IKClient()
        target_position = IK.compute_ik(xyz, quaternion, self.current_positions)

        if not target_position or len(target_position) < 6:
            print("IK computation failed. Skipping this target ", target_position)
            return

        if gripper_val is not None:
            self.move_gripper(gripper_val, -gripper_val)
            print("yey send gripper")

        self.set_parameters(target_position, movement_time, velocity, acceleration)
        self.move_to_target()

        rclpy.spin_once(self)


def sleep_pose():
    # x y z qx qy qz qw 
    return 0.12, 0.0, 0.08, 0.0, 0.27, 0.0, 0.96

def main():
    # instances
    rclpy.init()
    arm_control = WidowX250s()


    # set variables (const)
    x, y, z = 0.3, 0.0, 0.1
    
    # x BELOW 0.3 IS RISKY
    
    
    # x, y, z = get_tag_coords(arm_control)

    # cap.release()

    # print()
    # print("x" + str(x))
    # print("y" + str(y))
    # print("z" + str(z))
    # print()


    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    
    # x, y, z, qx, qy, qz, qw = sleep_pose()

    gripper_val = 0.035 # 0.035 to open, 0.0 to close

    # x, y, z, qx, qy, qz, qw = sleep_pose()
    
    # movement parameters
    movement_time = 2
    velocity = [0.5] * 6
    acceleration = [0.1] * 6

    
    arm_control.execute_target(xyz=[x, y, z], 
        quaternion=[qx, qy, qz, qw], 
        movement_time=movement_time, 
        velocity=velocity,
        acceleration=acceleration,
        gripper_val=0.02
    )
        

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()


'''debugging resetting gazebo sim'''
# ros2 service call /reset_world std_srvs/srv/Empty



# roll, pitch, yaw = compute_rpy_from_xyz(x, y, z)
# roll, pitch, yaw = compute_rpy_with_fixed_axis(np.array([1, 0, 0.5]), np.array([0, 0, 1]))
# roll, pitch, yaw = compute_rpy(direction=[0, 0, 1], fixed_axis='z', facing_axis='x', facing_hint=[1, 0, 0])
# roll, pitch, yaw = compute_rpy([1, 1, 0], fixed_axis='x')
# roll, pitch, yaw = compute_rpy([1, 1, 0], fixed_axis='y', up_axis='x')
# qx, qy, qz, qw = tf.quaternion_from_euler(roll, pitch, yaw)

# qx, qy, qz, qw = compute_quaternion([1, 0, 0], fixed_axis='y', up_axis='x')
# target_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# x, y, z, qx, qy, qz, qw = sleep_pose()
# x, y, z, qx, qy, qz, qw = to_pose()
