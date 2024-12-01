#!/usr/bin/env python3
#ros2 topic pub /target_movement geometry_msgs/Vector3 "{x: 1.0, y: 0.5, z: -0.5}"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from pymavlink import mavutil
from time import sleep, time
import numpy as np
import threading

class MavControlNode(Node):
    def __init__(self):
        super().__init__('mav_control_node')

        # Parameters
        self.declare_parameter('connection_string', 'udpin:0.0.0.0:14550')
        self.declare_parameter('num_thrusters', 8)
        self.declare_parameter('target_topic', '/target_movement')

        connection_string = self.get_parameter('connection_string').get_parameter_value().string_value
        self.num_thrusters = self.get_parameter('num_thrusters').get_parameter_value().integer_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value

        # MAVLink Connection
        self.vehicle = self.create_connection(connection_string)
        self.get_logger().info("MAVLink connection established.")

        # Backup thruster parameters
        self.backup_params = self.backup_thruster_params(self.num_thrusters)
        self.get_logger().info("Thruster parameters backed up.")

        # Enable passthrough mode (if needed)
        # self.enable_passthrough_mode(self.num_thrusters)
        # self.get_logger().info("RC Passthrough mode enabled.")

        # Set initial mode to ALT_HOLD
        self.set_mode('ALT_HOLD')

        # Arm the vehicle
        self.arm_vehicle()

        # Subscribe to target movements
        self.subscription = self.create_subscription(
            Vector3,
            target_topic,
            self.target_callback,
            10
        )
        self.get_logger().info(f"Subscribed to target topic: {target_topic}")

        # Mutex for thread safety
        self.lock = threading.Lock()

        # Start position monitoring thread
        self.monitoring_thread = threading.Thread(target=self.position_monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        self.get_logger().info("Started position monitoring thread.")

    def create_connection(self, connection_string):
        connection = mavutil.mavlink_connection(connection_string)
        connection.wait_heartbeat()
        self.get_logger().info("Heartbeat received.")
        return connection

    def set_mode(self, mode):
        mode_id = self.vehicle.mode_mapping().get(mode)
        if mode_id is None:
            self.get_logger().error(f"Unknown mode: {mode}")
            return
        self.vehicle.set_mode(mode_id)
        self.get_logger().info(f"Vehicle mode set to {mode}")

    def arm_vehicle(self):
        self.vehicle.arducopter_arm()
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle armed.")

    def disarm_vehicle(self):
        self.vehicle.arducopter_disarm()
        self.vehicle.motors_disarmed_wait()
        self.get_logger().info("Vehicle disarmed.")

    def set_servo_function(self, servo_num, function):
        param_name = f'SERVO{servo_num}_FUNCTION'
        self.vehicle.mav.param_set_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            param_name.encode('utf-8'),
            function,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        self.get_logger().debug(f"Set {param_name} to {function}")

    def enable_passthrough_mode(self, num_thrusters):
        for i in range(1, num_thrusters + 1):
            self.set_servo_function(i, 1)
            sleep(0.1)

    def disable_passthrough_mode(self, num_thrusters, backup_params):
        for i in range(1, num_thrusters + 1):
            self.set_servo_function(i, backup_params[i-1])
            sleep(0.1)

    def backup_thruster_params(self, num_thrusters):
        backup_params = []
        for i in range(1, num_thrusters + 1):
            param_name = f'SERVO{i}_FUNCTION'
            self.vehicle.mav.param_request_read_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                param_name.encode('utf-8'),
                -1
            )
            message = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if message:
                backup_params.append(int(message.param_value))
                self.get_logger().debug(f"Backed up {param_name}: {int(message.param_value)}")
            else:
                self.get_logger().warning(f"Failed to backup {param_name}. Using default value 0.")
                backup_params.append(0)
        return backup_params

    def fetch_current_state_local(self):
        self.get_logger().info('Fetching current state...')
        default_position = {'x': 0, 'y': 0, 'z': 0}
        default_orientation = {'yaw': 0}

        # Request LOCAL_POSITION_NED
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            0, 0, 0, 0, 0, 0, 0
        )
        position_message = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        if position_message:
            self.get_logger().info(f"Position received: x={position_message.x}, y={position_message.y}, z={position_message.z}")
            position = {'x': position_message.x, 'y': position_message.y, 'z': position_message.z}
        else:
            self.get_logger().warning("Failed to receive position message.")
            position = default_position

        # Request ATTITUDE
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            0, 0, 0, 0, 0, 0, 0
        )
        attitude_message = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        if attitude_message:
            self.get_logger().info(f"Attitude received: yaw={attitude_message.yaw}")
            orientation = {'yaw': attitude_message.yaw}
        else:
            self.get_logger().warning("Failed to receive attitude message.")
            orientation = default_orientation

        self.get_logger().info("--------------------------------------")

        self.get_logger().info(f"x: {position['x']}")
        self.get_logger().info(f"x: {position['x']}, y: {position['y']}, z: {position['z']}, yaw: {orientation['yaw']}")
        return {
            'x': position['x'],
            'y': position['y'],
            'z': position['z'],
            'yaw': orientation['yaw']
        }

    def wait_for_valid_position(self, max_attempts=200):
        for attempt in range(max_attempts):
            current_state = self.fetch_current_state_local()
            if current_state['x'] != 0 or current_state['y'] != 0 or current_state['z'] != 0:
                return current_state
            self.get_logger().info(f"Invalid position data, retrying... ({attempt + 1}/{max_attempts})")
            sleep(1)
        raise TimeoutError("Failed to get a valid position")

    def set_guided_mode_hold_position_local(self):
        self.get_logger().info("Setting GUIDED mode and holding position...")
        current_state = self.wait_for_valid_position()

        # Set to GUIDED mode
        self.set_mode('GUIDED')

        # Send position target to hold current position
        self.vehicle.mav.set_position_target_local_ned_send(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            current_state['x'],
            current_state['y'],
            current_state['z'],
            0, 0, 0,
            0, 0, 0,
            current_state['yaw'],
            0
        )
        self.get_logger().info(f"Holding position at x={current_state['x']}, y={current_state['y']}, z={current_state['z']}, yaw={current_state['yaw']}")
        return current_state

    def send_position_request(self, x, y, z):
        type_mask = 0b0000111111111000  # Only positions
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        self.vehicle.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.vehicle.target_system,  # Target system
            self.vehicle.target_component,  # Target component
            coordinate_frame,  # Coordinate frame
            type_mask,  # Type mask
            x, y, z,  # Positions in NED frame
            0, 0, 0,  # Velocities (not used)
            0, 0, 0,  # Accelerations (not used)
            0, 0  # Yaw and yaw rate (not used)
        )
        self.get_logger().info(f"Position request sent: x={x}, y={y}, z={z}")

    def wait_until_position_reached(self, target_position, margin=0.2, timeout=60):
        """
        Wait until the vehicle reaches the target position within a margin.
        """
        self.get_logger().info("Waiting to reach target position...")
        start_time = time()
        while time() - start_time < timeout:
            current_state = self.fetch_current_state_local()
            dx = current_state['x'] - target_position['x']
            dy = current_state['y'] - target_position['y']
            dz = current_state['z'] - target_position['z']
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            self.get_logger().info(f"Current distance to target: {distance:.2f} meters")
            if distance <= margin:
                self.get_logger().info("Target position reached within margin.")
                return current_state
            sleep(1)
        self.get_logger().warning("Timeout exceeded while waiting for target position.")
        return current_state  # Return last known state even if timeout

    def execute_movement(self, delta_x, delta_y, delta_z):
        with self.lock:
            try:
                # Switch to GUIDED mode
                self.set_mode('GUIDED')

                # Fetch current position
                current_state = self.wait_for_valid_position()

                # Calculate new target position
                new_x = current_state['x'] + delta_x
                new_y = current_state['y'] + delta_y
                new_z = current_state['z'] + delta_z

                # Send position request
                self.send_position_request(new_x, new_y, new_z)

                # Wait until position is reached
                target_position = {'x': new_x, 'y': new_y, 'z': new_z}
                self.wait_until_position_reached(target_position)

                # Switch back to ALT_HOLD mode
                self.set_mode('ALT_HOLD')

            except Exception as e:
                self.get_logger().error(f"Error during movement execution: {e}")
                self.disarm_vehicle()

    def target_callback(self, msg: Vector3):
        self.get_logger().info(f"Received target movement: delta_x={msg.x}, delta_y={msg.y}, delta_z={msg.z}")
        # Execute the movement in a separate thread to avoid blocking callbacks
        movement_thread = threading.Thread(target=self.execute_movement, args=(msg.x, msg.y, msg.z))
        movement_thread.start()

    def backup_thruster_params(self, num_thrusters):
        backup_params = []
        for i in range(1, num_thrusters + 1):
            param_name = f'SERVO{i}_FUNCTION'
            self.vehicle.mav.param_request_read_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                param_name.encode('utf-8'),
                -1
            )
            message = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if message:
                backup_params.append(int(message.param_value))
                self.get_logger().debug(f"Backed up {param_name}: {int(message.param_value)}")
            else:
                self.get_logger().warning(f"Failed to backup {param_name}. Using default value 0.")
                backup_params.append(0)
        return backup_params

    def position_monitoring_loop(self):
        """
        Continuously fetch and log the current position at regular intervals.
        """
        while rclpy.ok():
            with self.lock:
                try:
                    current_state = self.fetch_current_state_local()
                    self.get_logger().info(f"Current State - x: {current_state['x']}, y: {current_state['y']}, z: {current_state['z']}, yaw: {current_state['yaw']}")
                except Exception as e:
                    self.get_logger().error(f"Error fetching current state: {e}")
            sleep(1)  # Fetch every 1 second

    def __del__(self):
        # Cleanup actions when node is destroyed
        try:
            self.disarm_vehicle()
            # Disable passthrough mode if it was enabled
            # self.disable_passthrough_mode(self.num_thrusters, self.backup_params)
            self.get_logger().info("Cleanup completed.")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MavControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
