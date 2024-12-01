# Copyright 2024, Markus Buchholz

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
import threading
import time
import sys
import termios
import tty
import select

class BlueBoatKeyController(Node):

    def __init__(self):
        super().__init__('blue_boat_key_controller')

        self.callback_group = ReentrantCallbackGroup()
        
        self.motor_port_publisher = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_port_joint/cmd_thrust',
            10,
            callback_group=self.callback_group
        )
        
        self.motor_stbd_publisher = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_stbd_joint/cmd_thrust',
            10,
            callback_group=self.callback_group
        )

        self.port_thrust = 0.0
        self.stbd_thrust = 0.0

        self.lock = threading.Lock()

        self.get_logger().info('BlueBoatKeyController initialized')
        
        self.running = True

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()
        
        self.update_thread = threading.Thread(target=self.update_thrust)
        self.update_thread.start()

    def update_thrust(self):
        while self.running:
            with self.lock:
                self.motor_port_publisher.publish(Float64(data=self.port_thrust))
                self.motor_stbd_publisher.publish(Float64(data=self.stbd_thrust))
                #self.get_logger().info(f'port thrust : {self.port_thrust}')
                #self.get_logger().info(f'stbd thrust : {self.stbd_thrust}')
            time.sleep(0.1)

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(3)
                    with self.lock:
                        if key == '\x1b[A':  # Up arrow
                            self.port_thrust += 1.0
                            self.stbd_thrust += 1.0
                        elif key == '\x1b[B':  # Down arrow
                            self.port_thrust -= 1.0
                            self.stbd_thrust -= 1.0
                        elif key == '\x1b[D':  # Left arrow
                            self.port_thrust -= 1.0
                            self.stbd_thrust += 1.0
                        elif key == '\x1b[C':  # Right arrow
                            self.port_thrust += 1.0
                            self.stbd_thrust -= 1.0

                        self.port_thrust = max(min(self.port_thrust, 15.0), -15.0)
                        self.stbd_thrust = max(min(self.stbd_thrust, 15.0), -15.0)
        except Exception as e:
            self.get_logger().error(f'Error in keyboard listener: {e}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        self.running = False
        self.keyboard_thread.join()
        self.update_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    blue_boat_key_controller = BlueBoatKeyController()

    try:
        rclpy.spin(blue_boat_key_controller)
    except KeyboardInterrupt:
        blue_boat_key_controller.get_logger().info('Keyboard Interrupt (CTRL+C)')

    blue_boat_key_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
