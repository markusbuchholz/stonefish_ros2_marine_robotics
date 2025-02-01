#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile
from rclpy.time import Time


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry,
            '/bluerov/navigator/odometry',
            self.callback,
            qos_profile)
        self.br = TransformBroadcaster(self)

    def callback(self, msg):
        t = TransformStamped()

        t.header.stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec).to_msg()
        t.header.frame_id = 'world_ned'
        t.child_frame_id = 'bluerov/base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

