#!/usr/bin/env python3

## Jonatan Scharff Willners @2023
## Markus Buchholz @2024 @ROS 2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor
import random


class BridgeNode(Node):

    def __init__(self):
        super().__init__('bridge_node')
        self.conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550') # SET THIS FOR SITL (GAZEBO)
        #self.conn = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
        self.statusFlagPrint = True
        self.conn.wait_heartbeat()
        

        match_count = 0
        while match_count < 2:
            try:
                print(self.conn.recv_match().to_dict())
                match_count += 1
            except Exception as e:
                print("Error while receiving match:", e)
                time.sleep(0.5)
        


        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )
        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30
        )
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
        self.timer = self.create_timer(1.0 / 30, self.update) # Collect data at 30Hz
        
        self.data = {}
      

    
    def send_heartbeat(self, MAVTYPE=12, MAVAUTOPILOT=8, MAVMODE=128, MAVSTATE_A=4, MAVSTATE_B=0):
        self.conn.mav.heartbeat_send(MAVTYPE, MAVAUTOPILOT, MAVMODE, MAVSTATE_A, MAVSTATE_B)    

    def get_data(self):
  
        return self.data

    def get_all_msgs(self):

        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        #print("update!!!!")
  
        msgs = self.get_all_msgs()
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()


    def set_mode(self, mode):

        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print("Unknown mode : {}".format(mode))
            print("Try:", list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)

    def decode_mode(self, base_mode, custom_mode):

        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, "MANUAL"],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, "STABILIZE"],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, "GUIDED"],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, "AUTO"],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, "TEST"],
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_stabilize_mode(self):
        """Set stabilize mode"""
        mode = 'STABILIZE'
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id  # 0 for STABILIZE mode
        )
    def set_alt_hold_mode(self):
        mode = 'ALT_HOLD'
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id  # 2 for ALT_HOLD mode
        )
    def set_guided_mode(self):
        """Set guided mode"""
        mode = 'GUIDED'
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.mav.set_mode_send(self.conn.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)  # 4 GUIDED mode
     

    def fetch_current_state_local(self):
        self.get_logger().info('FETCHING')
        # Default values if data is missing
        default_position = {'x': 0, 'y': 0, 'z': 0}
        default_orientation = {'yaw': 0}

        # Fetch position from LOCAL_POSITION_NED if available
        position = self.data.get('LOCAL_POSITION_NED', default_position)
        x = position.get('x', 0)
        y = position.get('y', 0)
        z = position.get('z', 0)

        # Fetch orientation from ATTITUDE if available
        orientation = self.data.get('ATTITUDE', default_orientation)
        yaw = orientation.get('yaw', 0)
        
        # Return the current state
        return {
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw
        }
 
    def set_guided_mode_hold_position_local(self):
        current_state = self.fetch_current_state_local()

        # Switch to Guided mode
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode
        )

        self.conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (0 to use system time)
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local frame
            0b0000111111111000,  # Type mask (specifying to use x, y, z position and yaw angle)
            current_state['x'],  # X Position in NED frame
            current_state['y'],  # Y Position in NED frame
            current_state['z'],  # Z Position in NED frame
            0, 0, 0,  # X, Y, Z velocity in m/s (not used)
            0, 0, 0,  # X, Y, Z acceleration (not used)
            0,  # Yaw angle in radians
            0 )
        

        yaw = current_state['yaw']
        q_w = np.cos(yaw / 2)
        q_x = 0
        q_y = 0
        q_z = np.sin(yaw / 2)
        q = [q_w, q_x, q_y, q_z]
        
        yaw_rate=0.0
        use_yaw_rate=False
        type_mask = 0b00000111 if not use_yaw_rate else 0b0000001
         
        try:
            self.conn.mav.set_attitude_target_send(
                time_boot_ms=0,
                target_system=self.conn.target_system,
                target_component=self.conn.target_component,
                type_mask=type_mask,
                q=q,
                body_roll_rate=0,
                body_pitch_rate=0,
                body_yaw_rate=yaw_rate if use_yaw_rate else 0,
                thrust=0.5
            )
        except Exception as e:
            print(f"Error occurred while sending attitude target: {e}")


    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):

        self.conn.mav.command_long_send(
            self.conn.target_system,  # target system
            self.conn.target_component,  # target component
            command,  # mavlink command
            confirmation,  # confirmation
            params[0],  # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6],
        )

    def set_position_target_local_ned(self, param):

        time_boot_ms = 0  # Timestamp (milliseconds, not used)
        target_system = self.conn.target_system
        target_component = self.conn.target_component
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED  # Use local North-East-Down frame
        type_mask = 0b0000110111111000  # Position only

        # Set command to move the vehicle to a specified position
        self.conn.mav.set_position_target_local_ned_send(
            time_boot_ms,
            target_system,
            target_component,
            coordinate_frame,
            type_mask,
            param[0],  # North position in meters
            -param[1],  # East position in meters (check if inversion is needed based on your coordinate system understanding)
            -param[2],  # Down position in meters (inverted as positive Down is downward in NED)
            0,  # X velocity (not used)
            0,  # Y velocity (not used)
            0,  # Z velocity (not used)
            0,  # X acceleration (not used)
            0,  # Y acceleration (not used)
            0,  # Z acceleration (not used)
            0,  # Yaw angle (not used)
            0   # Yaw rate (not used)
        )



    def set_attitude_target(self, param=[]):
        print("....  requesting the target attitude .....")
    
        yaw_rate=0.0
        use_yaw_rate=False
        type_mask = 0b00000111 if not use_yaw_rate else 0b00000011
        
        q = param[0]
        
        try:
            self.conn.mav.set_attitude_target_send(
                time_boot_ms=0,
                target_system=self.conn.target_system,
                target_component=self.conn.target_component,
                type_mask=type_mask,
                q=q,
                body_roll_rate=0,
                body_pitch_rate=0,
                body_yaw_rate=yaw_rate if use_yaw_rate else 0,
                thrust=0.5
            )
        except Exception as e:
            print(f"Error occurred while sending attitude target: {e}")


    def set_servo_pwm(self, id, pwm=0):

        mavutil.mavfile.set_servo(self.conn, id, pwm)

    def set_rc_channel_pwm(self, id, pwm=0):

        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id] = pwm
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,  # target_system
            self.conn.target_component,  # target_component
            *rc_channel_values
        )  # RC channel list, in microseconds.



    def set_rc_channels_pwm(self, vals):

        rc_channel_values = [int(val) for val in vals[:8]]
        #print("values:", rc_channel_values)

        while len(rc_channel_values) < 8:
            rc_channel_values.append(0)

        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,  # target_system
            self.conn.target_component,  # target_component
            rc_channel_values[0],
            rc_channel_values[1],
            rc_channel_values[2],
            rc_channel_values[3],
            rc_channel_values[4],
            rc_channel_values[5],
            rc_channel_values[6],
            rc_channel_values[7]
        )
        

    def arm_throttle(self, arm_throttle):

        if arm_throttle:
            self.conn.arducopter_arm()
        else:
      
            self.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ],
            )

    def request_message_interval(self, message_id, frequency_hz):

        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,  # The MAVLink message ID
            1e6
            / frequency_hz,  # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0,  # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
            0,
            0,
            0,
            0,
        )

    # https://github.com/thien94/vision_to_mavros/blob/master/scripts/t265_to_mavlink.py
    # https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-2-complete-installation-and-indoor-non-gps-flights/43405
    # https://ardupilot.org/blimp/docs/common-vicon-for-nongps-navigation.html
    # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
    # https://www.ardusub.com/developers/pymavlink.html#send-gps-position
    def send_global_vision_position_estimate(self, x, y):
        current_time_us = int(time.time() * 1e6)  # Unix time in microseconds
        
        x = random.uniform(-10.05, 10.05)  # Increment/decrement within +/- 5 cm
        y = random.uniform(-10.05, 10.05)
        z = random.uniform(-10.05, 10.05)
        roll = random.uniform(-10.001, 10.001)  # Small radian fluctuation
        pitch = random.uniform(-10.001, 10.001)
        yaw = random.uniform(-10.001, 10.001)

        # Covariance (simulated as a simple scaled identity matrix for demonstration)
        tracker_confidence = 3  # Simulate a tracking confidence level (1-3)
        cov_scale = pow(10, 3 - tracker_confidence)
        covariance = [0.01 * cov_scale] * 21  # Simplified diagonal covariance
        
        print (" : ", x, " : ", y," : ", z, " : ",roll," : ", pitch," : ", yaw)
        self.conn.mav.vision_position_estimate_send(
            current_time_us,  # Timestamp (microseconds since UNIX epoch)
            x,                # Global X position
            y,                # Global Y position
            z,                # Global Z position
            roll,             # Roll angle in radians
            pitch,            # Pitch angle in radians
            yaw,              # Yaw angle in radians
            covariance        # Covariance matrix upper right triangular (first six rows of 6x6 matrix)
        )

    
def main(args=None):
    rclpy.init(args=args)
    bridge_node = BridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    
    try:
        executor.spin()  
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()