import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ASVController(Node):
    def __init__(self):
        super().__init__('asv_controller')

        # Publishers for the two thrusters
        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)

        # Subscriber to cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute the thrust for each motor
        # Assuming a simple differential thrust model
        k_linear = 1.0  # Gain for linear velocity
        k_angular = 1.0  # Gain for angular velocity

        thrust_port = k_linear * linear_x - k_angular * angular_z
        thrust_stbd = k_linear * linear_x + k_angular * angular_z

        # Create Float64 messages for the thrusts
        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd

        # Publish the thrust commands
        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ASVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
