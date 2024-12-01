from pymavlink import mavutil
import time

class BoatController:
    def __init__(self, connection_string):
        # Create the connection
        self.conn = mavutil.mavlink_connection(connection_string)
        # Wait for a heartbeat before sending commands
        self.conn.wait_heartbeat()

    def set_parameter(self, param_id, param_value):
        """Set a parameter on the vehicle"""
        param_id_str = param_id if isinstance(param_id, str) else param_id.decode('ascii')
        param_id_str = param_id_str.encode('utf-8')

        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            param_id_str,
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        
        # Wait for the parameter to be acknowledged
        print(f"Setting parameter {param_id} to {param_value}")
        while True:
            message = self.conn.recv_match(type='PARAM_VALUE', blocking=True)
            if message:
                print(f"Received parameter: {message.to_dict()}")
                if message.param_id == param_id:
                    print(f"Parameter {param_id} set to {param_value}")
                    break

    def arm_vehicle(self):
        # Try to arm the vehicle
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        # Wait and check for arming confirmation
        print("Attempting to arm the vehicle...")
        timeout = time.time() + 10  # 10 second timeout for arming
        armed = False

        while not armed and time.time() < timeout:
            message = self.conn.recv_match(type='HEARTBEAT', blocking=True)
            if message:
                print(message.to_dict())
                armed = message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                if armed:
                    print('Vehicle is armed!')
                    break

        if not armed:
            print('Failed to arm the vehicle. Check pre-arm conditions and messages.')

    def set_guided_mode(self):
        """Set guided mode"""
        mode = 'GUIDED'
        mode_id = self.conn.mode_mapping()[mode]

        # Print current status before setting mode
        self.print_status("before setting to GUIDED")

        # Set mode to GUIDED
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        # Print status after setting mode
        self.print_status("after setting to GUIDED")

    def set_auto_mode(self):
        """Set auto mode"""
        mode = 'AUTO'
        mode_id = self.conn.mode_mapping()[mode]

        # Print current status before setting mode
        self.print_status("before setting to AUTO")

        # Set mode to AUTO
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        # Print status after setting mode
        self.print_status("after setting to AUTO")

    def print_status(self, description):
        print(f"Current status {description}:")
        message = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if message:
            print(message.to_dict())
        else:
            print("No heartbeat message received.")

    def set_home_position(self):
        """Set home position to a dummy position"""
        print("Setting dummy home position...")
        dummy_lat = 47.397742  # Example latitude
        dummy_lon = 8.545594  # Example longitude
        dummy_alt = 488.0  # Example altitude in meters

        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,  # Use current position
            0, 0, 0, 0,
            dummy_lat,
            dummy_lon,
            dummy_alt)
        
        # Wait for home position to be acknowledged
        timeout = time.time() + 5
        while time.time() < timeout:
            message = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
            if message and message.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
                if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Home position set successfully.")
                    return True
                else:
                    print(f"Failed to set home position: {message.to_dict()}")
                    return False
        print("No acknowledgment received for setting home position.")
        return False

    def upload_mission(self, waypoints):
        """Upload a mission to the vehicle"""
        print("Uploading mission...")
        # Clear existing mission
        self.conn.mav.mission_clear_all_send(self.conn.target_system, self.conn.target_component)

        # Send the mission count
        self.conn.mav.mission_count_send(self.conn.target_system, self.conn.target_component, len(waypoints))

        for seq, waypoint in enumerate(waypoints):
            self.conn.mav.mission_item_int_send(
                self.conn.target_system,
                self.conn.target_component,
                seq,
                waypoint['frame'],
                waypoint['command'],
                0,  # current - set to 0 as we are uploading the whole mission
                1,  # autocontinue
                waypoint['param1'],
                waypoint['param2'],
                waypoint['param3'],
                waypoint['param4'],
                int(waypoint['x'] * 1e7),  # Convert to int as required by MAVLink
                int(waypoint['y'] * 1e7),  # Convert to int as required by MAVLink
                int(waypoint['z']),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )

        # Wait for mission ack
        timeout = time.time() + 10
        while time.time() < timeout:
            message = self.conn.recv_match(type='MISSION_ACK', blocking=True)
            if message:
                print(f"Mission acknowledgment: {message.to_dict()}")
                if message.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("Mission uploaded successfully.")
                    return True
                else:
                    print(f"Mission upload failed: {message.to_dict()}")
                    return False
        print("No acknowledgment received for mission upload.")
        return False

    def send_move_command_global(self, lat, lon, alt):
        """Send move command to boat using global coordinates"""
        print(f"Sending move command to lat={lat}, lon={lon}, alt={alt}")

        self.conn.mav.set_position_target_global_int_send(
            0,  # time_boot_ms (not used)
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # coordinate frame - mav_frame global
            3580,  # type mask - use position
            lat,  # latitude * 1e7
            lon,  # longitude * 1e7
            alt,  # altitude in m above sea level, home or terrain
            0,  # x vel in m/s positive is north
            0,  # y vel
            0,  # z vel
            0,  # accel x
            0,  # accel y
            0,  # accel z
            0,  # yaw in rad - 0 is forward
            0   # yaw rate rad/s
        )

        # Check for acknowledgment of the command
        timeout = time.time() + 5
        while time.time() < timeout:
            ack = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if ack:
                print(f"Command acknowledgment: {ack.to_dict()}")
                if ack.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Move command accepted.")
                    return True
                else:
                    print(f"Move command not acknowledged correctly: {ack.to_dict()}")
                    return False
        print("No acknowledgment received for move command.")
        return False

    def start_mission(self):
        """Starts the mission"""
        print("Starting mission...")
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,  # first mission item to execute
            0,  # first mission item to execute on restart
            0, 0, 0, 0, 0)

        # Wait for mission start acknowledgment
        timeout = time.time() + 5
        while time.time() < timeout:
            message = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
            if message and message.command == mavutil.mavlink.MAV_CMD_MISSION_START:
                if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Mission started successfully.")
                    return True
                else:
                    print(f"Failed to start mission: {message.to_dict()}")
                    return False
        print("No acknowledgment received for mission start.")
        return False

if __name__ == "__main__":
    boat_controller = BoatController('udpin:192.168.2.1:14550')
    
    # Disable all arming checks
    boat_controller.set_parameter('ARMING_CHECK', 0)
    
    boat_controller.arm_vehicle()

    # Set home position to a dummy position
    if not boat_controller.set_home_position():
        print("Failed to set home position, aborting mission setup.")
    else:
        # Ensure the boat is in GUIDED mode for movement commands
        boat_controller.set_guided_mode()

        # Send move command using global coordinates
        target_lat = 30391280  # Example latitude * 1e7
        target_lon = -97902180  # Example longitude * 1e7
        target_alt = -0.9  # Example altitude in meters above sea level

        if boat_controller.send_move_command_global(target_lat, target_lon, target_alt):
            print("Move command executed successfully.")
        else:
            print("Move command failed.")
        
        # Optionally, set the boat to AUTO mode and upload a mission if required
        boat_controller.set_auto_mode()
        waypoints = [
            {'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
             'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0, 'x': 47.397742, 'y': 8.545594, 'z': 10},
            # Add more waypoints as needed
        ]
        if boat_controller.upload_mission(waypoints):
            print("Mission uploaded and started successfully.")
            if boat_controller.start_mission():
                print("Mission started successfully.")
            else:
                print("Failed to start mission.")
        else:
            print("Failed to upload mission.")
