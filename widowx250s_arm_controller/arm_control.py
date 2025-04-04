import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion

import numpy as np
import tf_transformations as tf
from scipy.spatial.transform import Rotation as R

from interbotix_xs_msgs.msg import JointGroupCommand

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /compute_ik service...")

    def compute_ik(self, xyz, quat, current_state):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "interbotix_arm"
        request.ik_request.pose_stamped.header.frame_id = "world"
        request.ik_request.pose_stamped.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        request.ik_request.pose_stamped.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        request.ik_request.robot_state.joint_state = JointState()
        request.ik_request.robot_state.joint_state.name = [
            "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"
        ]
        request.ik_request.robot_state.joint_state.position = current_state

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            joint_solution = future.result().solution.joint_state.position[:6]
            self.get_logger().info(f"IK Computation Successful: {joint_solution}")
            return list(joint_solution)
        else:
            self.get_logger().error("IK Computation Failed!")
            return None

class Wx250sArmController(Node):
    def __init__(self, target_position=None, movement_time=4, velocity=None, acceleration=None):
        super().__init__('wx250s_arm_controller')

        self.target_position = target_position
        self.movement_time = movement_time
        self.velocity = velocity if velocity else [0.5] * 6
        self.acceleration = acceleration if acceleration else [0.2] * 6

        self.publisher = self.create_publisher(JointGroupCommand, '/wx250s/commands/joint_group', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/wx250s/joint_states',
            self.state_callback,
            10
        )

        self.current_positions = None

    def state_callback(self, msg):
        joint_name_order = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        joint_positions = dict(zip(msg.name, msg.position))

        try:
            self.current_positions = [joint_positions[joint] for joint in joint_name_order]
            self.get_logger().info(f"Current Arm Position: {self.current_positions}")
        except KeyError:
            self.get_logger().warn("Some expected joint names were not found.")
            return

        if self.target_position and self.has_reached_goal(self.current_positions, self.target_position):
            self.get_logger().info("Goal position reached.")
            rclpy.shutdown()

    def has_reached_goal(self, current, target, tolerance=0.01):
        return all(abs(c - t) < tolerance for c, t in zip(current, target))

    def move_to_target(self):
        if not self.target_position:
            self.get_logger().warn("Target position not set.")
            return

        cmd_msg = JointGroupCommand()
        cmd_msg.name = "arm"
        cmd_msg.cmd = self.target_position

        self.publisher.publish(cmd_msg)
        self.get_logger().info("Published joint group command to target.")


class Wx250sGripperController(Node):
    def __init__(self, *args, **kwargs):
        super().__init__('gripper_commander')
        self.publisher_ = self.create_publisher(JointTrajectory, '/wx250s/gripper_controller/joint_trajectory', 10)
        self.get_logger().info('GripperCommander initialized.')

    def move_gripper(self, left_position: float, right_position: float):
        traj = JointTrajectory()
        traj.joint_names = ['left_finger', 'right_finger']
        
        point = JointTrajectoryPoint()
        point.positions = [left_position, right_position]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        traj.points.append(point)
        self.publisher_.publish(traj)
        self.get_logger().info(f'Gripper command sent: left={left_position}, right={right_position}')


def compute_rpy_from_xyz(x, y, z):
    return 0.0, 0.0, 0.0

def main():
    rclpy.init()

    # Step 1: Start controller to get joint state
    arm_node = Wx250sArmController()
    while arm_node.current_positions is None or len(arm_node.current_positions) == 0:
        rclpy.spin_once(arm_node)
        print("Waiting for valid joint state...")

    print(f"Received current joint state: {arm_node.current_positions}")

    # Step 2: Compute IK from desired pose
    xyz = [0.34, 0.0, 0.13]

    roll, pitch, yaw = compute_rpy_from_xyz(*xyz)
    quat = tf.quaternion_from_euler(roll, pitch, yaw)

    ik_node = IKClient()
    target_position = ik_node.compute_ik(xyz, quat, arm_node.current_positions)

    if target_position is None:
        print("IK failed. Exiting.")
        rclpy.shutdown()
        return

    print(f"Target Position from IK: {target_position}")

    # Step 3: Command movement
    move_node = Wx250sArmController(
        target_position=target_position,
        movement_time=4,
        velocity=[0.5] * 6,
        acceleration=[0.2] * 6
    )

    move_node.move_to_target()
    rclpy.spin(move_node)
    rclpy.shutdown()


main()


# def compute_rpy_from_xyz(x, y, z):
#     # Assume Z-axis is pointing in the direction of (x, y, z)
#     z_axis = np.array([x, y, z])
#     z_axis = z_axis / np.linalg.norm(z_axis)  # Normalize
    
#     # Assume X-axis is fixed (you may need to adjust based on your setup)
#     x_axis = np.array([1, 0, 0])
    
#     # Compute Y-axis as the cross product
#     y_axis = np.cross(z_axis, x_axis)
#     y_axis = y_axis / np.linalg.norm(y_axis)
    
#     # Compute corrected X-axis
#     x_axis = np.cross(y_axis, z_axis)
    
#     # Construct rotation matrix
#     R_mat = np.vstack([x_axis, y_axis, z_axis]).T
    
#     # Convert to roll-pitch-yaw
#     rpy = R.from_matrix(R_mat).as_euler('xyz', degrees=False)
    
#     return rpy[0], rpy[1], rpy[2]

# def compute_rpy_with_fixed_axis(direction: np.ndarray, fix_axis: np.ndarray, up_direction: np.ndarray = np.array([0, 0, 1])):
#     """
#     direction: 3D vector to align the fixed axis to (e.g., [1, 2, 0])
#     fix_axis: which axis to align (e.g., [1,0,0] for X-axis, [0,1,0] for Y-axis, [0,0,1] for Z-axis)
#     up_direction: preferred up direction (default: [0,0,1] to stay parallel to ground)
#     """
#     direction = direction / np.linalg.norm(direction)
    
#     # Determine which axis is being fixed
#     if np.allclose(fix_axis, [1, 0, 0]):
#         x_axis = direction
#         z_axis = np.cross(x_axis, up_direction)
#         if np.linalg.norm(z_axis) < 1e-6:
#             z_axis = np.array([0, 0, 1])
#         z_axis = z_axis / np.linalg.norm(z_axis)
#         y_axis = np.cross(z_axis, x_axis)

#     elif np.allclose(fix_axis, [0, 1, 0]):
#         y_axis = direction
#         z_axis = np.cross(up_direction, y_axis)
#         if np.linalg.norm(z_axis) < 1e-6:
#             z_axis = np.array([0, 0, 1])
#         z_axis = z_axis / np.linalg.norm(z_axis)
#         x_axis = np.cross(y_axis, z_axis)

#     elif np.allclose(fix_axis, [0, 0, 1]):
#         z_axis = direction
#         x_axis = np.cross(up_direction, z_axis)
#         if np.linalg.norm(x_axis) < 1e-6:
#             x_axis = np.array([1, 0, 0])
#         x_axis = x_axis / np.linalg.norm(x_axis)
#         y_axis = np.cross(z_axis, x_axis)

#     else:
#         raise ValueError("fix_axis must be [1,0,0] or [0,1,0] or [0,0,1]")

#     # Construct rotation matrix
#     R_mat = np.vstack([x_axis, y_axis, z_axis]).T
#     rpy = R.from_matrix(R_mat).as_euler('xyz', degrees=False)
#     return rpy

# def compute_rpy(direction, fixed_axis='z', facing_axis='x', facing_hint=np.array([1, 0, 0])):
#     """
#     direction: target vector to point the fixed_axis at
#     fixed_axis: 'x', 'y', or 'z' (robot axis to align)
#     facing_axis: the other robot axis you want to keep aligned (e.g., X points forward)
#     facing_hint: world direction you'd prefer the facing_axis to align with (default: [1,0,0])
#     """
#     direction = np.array(direction) / np.linalg.norm(direction)
#     facing_hint = np.array(facing_hint)
#     if np.linalg.norm(facing_hint) < 1e-6:
#         raise ValueError("Facing hint must be a valid vector")

#     # Build a right-handed coordinate frame
#     axis_map = {'x': 0, 'y': 1, 'z': 2}
#     if fixed_axis not in axis_map or facing_axis not in axis_map:
#         raise ValueError("fixed_axis and facing_axis must be 'x', 'y', or 'z'")
#     if fixed_axis == facing_axis:
#         raise ValueError("fixed_axis and facing_axis must be different")

#     # Build axes
#     A = direction
#     B = np.cross(A, facing_hint)
#     if np.linalg.norm(B) < 1e-6:
#         # Facing hint is parallel to direction → ambiguous
#         B = np.array([0, 1, 0]) if fixed_axis != 'y' else np.array([1, 0, 0])
#     B = B / np.linalg.norm(B)
#     C = np.cross(A, B)

#     # Assign axes based on fixed_axis and facing_axis
#     axes = {'x': None, 'y': None, 'z': None}
#     axes[fixed_axis] = A
#     axes[facing_axis] = B
#     for key in axes:
#         if axes[key] is None:
#             axes[key] = C

#     R_mat = np.vstack([axes['x'], axes['y'], axes['z']]).T
#     return R.from_matrix(R_mat).as_euler('xyz')

# def compute_quaternion(direction,fixed_axis='z',up_axis='z',fallback_axis='x'):

#     direction = np.array(direction)
#     direction = direction / np.linalg.norm(direction)

#     axis_map = {'x': 0, 'y': 1, 'z': 2}
#     if fixed_axis not in axis_map or up_axis not in axis_map or fallback_axis not in axis_map:
#         raise ValueError("fixed_axis, up_axis, and fallback_axis must be 'x', 'y', or 'z'")

#     robot_axes = {'x': None, 'y': None, 'z': None}
#     world_up = np.eye(3)[:, axis_map[up_axis]]
#     fallback = np.eye(3)[:, axis_map[fallback_axis]]

#     A = direction
#     B = np.cross(world_up, A)

#     if np.linalg.norm(B) < 1e-6:
#         # Direction is parallel to up — use fallback
#         B = np.cross(fallback, A)
#     B = B / np.linalg.norm(B)
#     C = np.cross(A, B)

#     # Assign axes based on fixed_axis
#     robot_axes[fixed_axis] = A
#     # Pick one for secondary alignment
#     remaining_axes = [ax for ax in ['x', 'y', 'z'] if robot_axes[ax] is None]
#     robot_axes[remaining_axes[0]] = B
#     robot_axes[remaining_axes[1]] = C

#     R_mat = np.vstack([robot_axes['x'], robot_axes['y'], robot_axes['z']]).T
#     return R.from_matrix(R_mat).as_quat()  # [x, y, z, w]
