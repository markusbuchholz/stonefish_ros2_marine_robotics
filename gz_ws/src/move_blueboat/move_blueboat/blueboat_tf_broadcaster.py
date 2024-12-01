import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class BlueboatTFBroadcaster(Node):

    def __init__(self):
        super().__init__('blueboat_tf_broadcaster')
        self.subscription = self.create_subscription(
            JointState,
            '/world/waves/model/blueboat/joint_state',
            self.listener_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Blueboat TF Broadcaster has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received joint state message: {msg}")
        for i, joint_name in enumerate(msg.name):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.get_parent_frame(joint_name)
            t.child_frame_id = joint_name

            # For illustration, we assume translation and rotation based on joint position
            # These values should be set based on actual joint states and configuration
            t.transform.translation.x = 0.0  # Adjust as necessary
            t.transform.translation.y = 0.0  # Adjust as necessary
            t.transform.translation.z = 0.0  # Adjust as necessary

            # Assuming a fixed rotation, adjust based on actual joint state
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(f"Sent transform: {t}")

    def get_parent_frame(self, joint_name):
        if joint_name == 'imu_joint':
            return 'base_link'
        elif joint_name == 'navsat_joint':
            return 'imu_link'
        elif joint_name == 'motor_port_joint':
            return 'base_link'
        elif joint_name == 'motor_stbd_joint':
            return 'base_link'
        return 'base_link'  # Default parent frame

def main(args=None):
    rclpy.init(args=args)
    node = BlueboatTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
