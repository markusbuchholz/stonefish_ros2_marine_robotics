#!/usr/bin/env python3
# Markus Buchholz, 2025 (Modified for Blueboat)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Int32MultiArray  # Import for PWM data

from pymavlink import mavutil
import threading
import time
import numpy as np

class BlueboatROSInterface(Node):
    def __init__(self):
        super().__init__('blueboat_ros_interface')

        self.declare_parameter('mavlink_connection', 'udpin:0.0.0.0:14550')
        self.declare_parameter('data_stream_rate', 8)  
        self.declare_parameter('num_thrusters', 8)    

        mavlink_connection_str = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        data_stream_rate = self.get_parameter('data_stream_rate').get_parameter_value().integer_value
        self.num_thrusters = self.get_parameter('num_thrusters').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to ArduPilot via {mavlink_connection_str}...')
        self.conn = mavutil.mavlink_connection(mavlink_connection_str)
        self.conn.wait_heartbeat()
        self.get_logger().info(f'Heartbeat from system (system {self.conn.target_system} component {self.conn.target_component})')

        self.thrusterRanges = [1100.0, 1900.0]  # PWM range for thrusters
        self.thrusterInputRanges = [-3.0, 3.0]   # Control input range

        #self.backup_params = self.backup_thruster_params()

        #self.enable_passthrough_mode()

        self.set_stabilize_mode_and_arm()

        # self.request_mavlink_messages()

        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.servo_output_publisher = self.create_publisher(
            Int32MultiArray,
            '/blueboat/servo_outputs',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )

        self.cmd_vel_subscription = self.create_subscription(
            # Using /blueboat/cmd_vel instead of /bluerov2/cmd_vel
            Twist,
            '/blueboat/cmd_vel',
            self._cmd_vel_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
            callback_group=ReentrantCallbackGroup()
        )

        self.data_lock = threading.Lock()
        self.data = {}
        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        self.timer = self.create_timer(1.0 / 30.0, self._publish_servo_outputs)  # 30 Hz

        self.get_logger().info('Blueboat ROS Interface node initialized.')

    def backup_thruster_params(self):
        """Backup the current SERVO_FUNCTION parameters."""
        backup = []
        for i in range(1, self.num_thrusters + 1):
            param_name = f'SERVO{i}_FUNCTION'
            self.conn.mav.param_request_read_send(
                self.conn.target_system,
                self.conn.target_component,
                param_name.encode('utf-8'),
                -1
            )
            message = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if message:
                backup.append(int(message.param_value))
                self.get_logger().info(f'Backed up {param_name}: {int(message.param_value)}')
            else:
                backup.append(0)
                self.get_logger().warn(f'Failed to backup {param_name}, defaulting to 0')
        return backup

    def set_servo_function(self, servo_num, function):
        """Set the SERVO_FUNCTION parameter for a specific thruster."""
        param_name = f'SERVO{servo_num}_FUNCTION'
        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            param_name.encode('utf-8'),
            function,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        self.get_logger().info(f'Set {param_name} to {function}')
        time.sleep(0.1)  # Small delay to ensure the command is processed

    def enable_passthrough_mode(self):
        """Enable passthrough mode for all thrusters."""
        self.get_logger().info('Enabling passthrough mode for thrusters...')
        for i in range(1, self.num_thrusters + 1):
            self.set_servo_function(i, 1)  # 1 typically stands for passthrough
        self.get_logger().info('Passthrough mode enabled.')

    def disable_passthrough_mode(self):
        """Disable passthrough mode and restore original thruster functions."""
        self.get_logger().info('Disabling passthrough mode and restoring thruster functions...')
        for i in range(1, self.num_thrusters + 1):
            original_function = self.backup_params[i-1]
            self.set_servo_function(i, original_function)
        self.get_logger().info('Passthrough mode disabled and thruster functions restored.')

    def set_stabilize_mode_and_arm(self):
        """Set the vehicle's mode to STABILIZE and arm it."""
        self.get_logger().info('Setting STABILIZE mode...')
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0  # 0 typically corresponds to STABILIZE mode
        )
        time.sleep(1)  # Wait for mode to be set

        self.get_logger().info('Arming the vehicle...')
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # Arm
        )
        # Wait until vehicle is armed
        armed = False
        for _ in range(10):
            message = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if message and (message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                self.get_logger().info('Vehicle armed successfully.')
                break
            time.sleep(0.5)
        if not armed:
            self.get_logger().error('Failed to arm the vehicle.')
            raise Exception('Vehicle failed to arm.')

    def mavlink_listener(self):
        """Continuously listen for MAVLink messages and update data."""
        while rclpy.ok():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
                if msg:
                    msg_type = msg.get_type()
                    with self.data_lock:
                        self.data[msg_type] = msg.to_dict()
                    self.get_logger().debug(f'Received MAVLink message: {msg_type}')
            except Exception as e:
                self.get_logger().error(f'Error while receiving MAVLink message: {e}')
                time.sleep(0.1)

    def _publish_servo_outputs(self):
        """Extract SERVO_OUTPUT_RAW data and publish PWM values."""
        with self.data_lock:
            servo_output = self.data.get('SERVO_OUTPUT_RAW', None)

        if not servo_output:
            self.get_logger().warn('SERVO_OUTPUT_RAW message not received yet.')
            return

        # Extract servo1_raw to servo8_raw
        pwm_values = [
            servo_output.get('servo1_raw', 0),
            servo_output.get('servo2_raw', 0),
            servo_output.get('servo3_raw', 0),
            servo_output.get('servo4_raw', 0),
            servo_output.get('servo5_raw', 0),
            servo_output.get('servo6_raw', 0),
            servo_output.get('servo7_raw', 0),
            servo_output.get('servo8_raw', 0)
        ]

        pwm_msg = Int32MultiArray()
        pwm_msg.data = pwm_values
        self.servo_output_publisher.publish(pwm_msg)

        self.get_logger().debug(f'Published PWM Values: {pwm_values}')

    def mapRanges(self, value):
        """Map a value from thruster input range to PWM range and clip."""
        # Ensure input ranges are valid
        in_min, in_max = self.thrusterInputRanges
        out_min, out_max = self.thrusterRanges

        # Perform linear mapping
        mapped_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        # Clip the mapped value to thrusterRanges
        clipped_value = np.clip(mapped_value, out_min, out_max)

        return clipped_value

    def _cmd_vel_callback(self, msg):
        """Handle incoming velocity commands and translate to RC overrides for Blueboat."""
        override = []

        # Example provided by the user:
        # override.append(1500 - msg.linear.y)  # Pitch
        # override.append(self.mapRanges(msg.angular.x))
        # override.append(self.mapRanges(msg.linear.z))
        # override.append(3000 - self.mapRanges(msg.angular.z))  # Forward
        # override.append(self.mapRanges(msg.linear.x))
        # override.append(65535)  # Lateral
        # override.append(65535)  # light strength
        # override.append(65535)  # camera servo tilt

        # Adjusted for Blueboat with num_thrusters=4
        # Assuming thrusters 1-4 correspond to specific controls
        # Modify as per your specific thruster configuration
        
        try:
            override.append(1500 - msg.linear.y)  # Pitch
            override.append(self.mapRanges(msg.angular.x))  # 
            override.append(self.mapRanges(msg.linear.z))  # 
            override.append(3000 - self.mapRanges(msg.angular.z))  # Forward
            override.append(self.mapRanges(msg.linear.x))  # 
            override.append(65535)  # Lateral
            override.append(65535)  # light strength
            override.append(65535)  # camera servo tilt

            # Example mapping; adjust indices based on your thruster setup
            #pwm_pitch = int(1500 - 2 * msg.linear.y)
            #pwm_roll = self.mapRanges(msg.angular.x)
            #pwm_heave = self.mapRanges(msg.linear.z)
            #pwm_forward = int(3000 - self.mapRanges(msg.angular.z))  # Example mapping
            #pwm_lateral = self.mapRanges(msg.linear.x)

            # For Blueboat with 4 thrusters, you might map as follows:
            # Thruster 1: Forward/Reverse (pwm_forward)
            # Thruster 2: Lateral (pwm_lateral)
            # Thruster 3: Heave (pwm_heave)
            # Thruster 4: Pitch/Roll (combination or separate)

            # Construct PWM override list
            #override.append(pwm_forward)   # Thruster 1
            #override.append(pwm_lateral)   # Thruster 2
            #override.append(pwm_heave)     # Thruster 3
            #override.append(pwm_pitch)     # Thruster 4

            # If your Blueboat has exactly 4 thrusters, pad the remaining with 65535
            while len(override) < 8:
                override.append(65535)

            self.get_logger().debug(f'Sending RC Channels Override: {override}')
            self.set_rc_channels_pwm(override)
            self.get_logger().info(f'RC Channels Override sent: {override}')
        except Exception as e:
            self.get_logger().error(f'Failed to process cmd_vel: {e}')

    def set_rc_channels_pwm(self, vals):
        """Override RC channels with provided PWM values."""
        # Ensure exactly 8 PWM values
        rc_channel_values = [int(val) for val in vals[:8]]
        while len(rc_channel_values) < 8:
            rc_channel_values.append(65535)  # Typically 65535 means no change

        self.get_logger().debug(f'Sending RC Channels Override: {rc_channel_values}')

        try:
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,      # target_system
                self.conn.target_component,   # target_component
                *rc_channel_values
            )
            self.get_logger().info(f'RC Channels Override sent: {rc_channel_values}')
        except Exception as e:
            self.get_logger().error(f'Failed to send RC override: {e}')

    def shutdown(self):
        """Safely shutdown the node, disarm the vehicle, and disable passthrough."""
        self.get_logger().info('Shutting down Blueboat ROS Interface node...')
        try:
            # Disarm the vehicle
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0  # Disarm
            )
            self.get_logger().info('Vehicle disarmed.')

            # Disable passthrough mode
            self.disable_passthrough_mode()

            # Close MAVLink connection
            self.conn.close()
            self.get_logger().info('MAVLink connection closed.')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BlueboatROSInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Initiating shutdown...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
