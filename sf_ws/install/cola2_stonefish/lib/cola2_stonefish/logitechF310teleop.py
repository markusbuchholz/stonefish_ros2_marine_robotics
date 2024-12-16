#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        # Set up publishers and subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rov_control_pub = self.create_publisher(Float64MultiArray, '/bluerov/controller/thruster_setpoints_sim', 10)

        # Specify which joystick controls vertical and horizontal motion
        self.vertical_joystick_index = 1  # Assuming left analog stick
        self.horizontal_joystick_index = 0  # Assuming left analog stick

        # Specify which joystick controls forward/backward and turning motion
        self.forward_backward_joystick_index = 4  # Assuming right analog stick
        self.turn_left_right_joystick_index = 3  # Assuming right analog stick

        # Initialize the last joystick ID
        self.last_joystick_id = None

        self.depth_control_mode = False

    def joy_callback(self, data):
        # Extract joystick values
        axes = data.axes
        buttons = data.buttons

        # Check if R2 and L2 are pressed simultaneously
        if buttons[4] == 1 and buttons[5] == 1:
            # Special case: R2 and L2 pressed simultaneously
            if self.depth_control_mode:
                self.depth_control_mode = False
                self.get_logger().info("DEPTH CONTROL MODE DISABLED!")
            else:
                self.get_logger().info("DEPTH CONTROL MODE ENABLED!")
                rov_control_msg = Float64MultiArray()
                rov_control_msg.data = [0.0, 0.0, 0.0, 0.0, 0.4, 0.4]  # Adjust as needed
                self.depth_control_mode = True

        else:
            # Map left analog stick to vertical motion (up/down) and horizontal motion (left/right)
            vertical = axes[self.vertical_joystick_index]
            horizontal = axes[self.horizontal_joystick_index]
            side = axes[self.turn_left_right_joystick_index]
            forward_backward = axes[self.forward_backward_joystick_index]
            depth = 0
            if self.depth_control_mode or forward_backward != 0 or side != 0:
                depth = 0.0
            else:
                depth = vertical * (-1)
            # Create a Float64MultiArray message with 6 values
            if side < 0 or horizontal < 0:
                rov_control_msg = Float64MultiArray()
                rov_control_msg.data = [
                    horizontal * (-1),
                    side,
                    forward_backward,
                    horizontal * (-1) + side + forward_backward,
                    depth,
                    depth
                ]
            else:
                rov_control_msg = Float64MultiArray()
                rov_control_msg.data = [
                    (-1) * side,
                    horizontal,
                    horizontal + (-1) * side + forward_backward,
                    forward_backward,
                    depth,
                    depth
                ]
            
        # Publish the control message
        self.rov_control_pub.publish(rov_control_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = JoystickController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

