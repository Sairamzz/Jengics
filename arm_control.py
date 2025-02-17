import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class Wx250sArmController(Node):
    def __init__(self):
        super().__init__('wx250s_arm_control')

        # Publisher for sending movement commands
        self.publisher = self.create_publisher(JointTrajectory, '/wx250s/arm_controller/joint_trajectory', 10)

        # Subscriber to read current arm state
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/wx250s/arm_controller/controller_state',
            self.state_callback,
            10
        )

        # Movement positions
        self.positions_list = [
            [0.5, -1.0, 1.0, -0.5, 0.3, 1.5],  # Position 1
            [-0.5, 1.0, -1.0, 0.5, -0.3, -1.5]  # Position 2
        ]
        self.current_index = 0  # Toggle between positions
        self.current_positions = None  # Store real-time positions from feedback

        # Start loop
        self.timer = self.create_timer(3.0, self.send_command)  # Send commands every 3 seconds

    def state_callback(self, msg):
        """ Reads the current joint positions from the arm state feedback. """
        self.current_positions = msg.actual.positions
        self.get_logger().info(f"Current Arm Position: {self.current_positions}")

    def send_command(self):
        """ Publishes a movement command to the arm. """
        if self.current_positions is None:
            self.get_logger().warn("Waiting for arm state...")
            return  # Skip sending commands until we get the first state

        traj = JointTrajectory()
        traj.joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]

        point = JointTrajectoryPoint()
        point.positions = self.positions_list[self.current_index]  # Move to new position
        point.time_from_start = Duration(sec=2)  # Move over 2 seconds

        traj.points.append(point)

        # Publish the movement command
        self.publisher.publish(traj)
        self.get_logger().info(f"Sent movement command to position {self.current_index + 1}")

        # Toggle between positions
        self.current_index = (self.current_index + 1) % len(self.positions_list)

def main():
    rclpy.init()
    node = Wx250sArmController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

