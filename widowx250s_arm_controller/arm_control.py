import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from jacobian import inverse_kinematics

class Wx250sArmController(Node):
    def __init__(self, target_position, movement_time, velocity=None, acceleration=None):
        super().__init__('wx250s_arm_control')


        self.publisher = self.create_publisher(JointTrajectory, '/wx250s/arm_controller/joint_trajectory', 10)

        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/wx250s/arm_controller/controller_state',
            self.state_callback,
            10
        )

        self.target_position = target_position
        self.movement_time = movement_time
        self.velocity = velocity if velocity else [0.0] * len(target_position)
        self.acceleration = acceleration if acceleration else [0.0] * len(target_position)
        self.current_positions = None

    def state_callback(self, msg):
        """ Reads the current joint positions from the arm state feedback. """
        self.current_positions = msg.actual
        self.get_logger().info(f"Current Arm Position: {self.current_positions}")
        
        if self.current_positions:
            self.move_to_target()

    def move_to_target(self):
        """ Publishes a movement command to move the arm to the target position and then stops. """
        traj = JointTrajectory()
        traj.joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]

        point = JointTrajectoryPoint()
        point.positions = self.target_position
        point.velocities = self.velocity
        point.accelerations = self.acceleration
        point.time_from_start = Duration(sec=self.movement_time)

        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info("Sent movement command to target position. Stopping execution.")
        
        self.destroy_node()


def main():
    rclpy.init()

    x = 1
    y = 1
    z = 1

    target_position, movement_time, velocity, acceleration = inverse_kinematics(x, y, z)
    
    node = Wx250sArmController(target_position, movement_time, velocity, acceleration)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()