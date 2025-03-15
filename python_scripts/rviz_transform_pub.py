import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class JengaTransformPublisher(Node):
    def __init__(self):
        super().__init__('jenga_transform_publisher')

        # Create a Static Transform Broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Define the transforms
        transforms = [
            ("world", "base_link", 0.5, 0, 0, 0, 0, 0, 1),
            ("base_link", "jenga_base_link", 0, 0, 0, 0, 0, 0, 1),
            ("base_link", "jenga_rotator_link", 0, 0, 0.04, 0, 0, 0, 1),
        ]

        # Publish each transform
        for parent, child, x, y, z, qx, qy, qz, qw in transforms:
            self.publish_static_transform(parent, child, x, y, z, qx, qy, qz, qw)

    def publish_static_transform(self, parent, child, x, y, z, qx, qy, qz, qw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static transform from {parent} to {child}")

def main(args=None):
    rclpy.init(args=args)
    node = JengaTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

