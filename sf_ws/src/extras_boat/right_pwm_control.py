import time
from pymavlink import mavutil

class BoatController:
    def __init__(self, connection_string):
        self.conn = mavutil.mavlink_connection(connection_string)
        self.conn.wait_heartbeat()
        print("Heartbeat received.")

    def set_parameter(self, param_id, param_value):
        param_id_str = param_id.encode('utf-8')
        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            param_id_str,
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        print(f"Setting parameter {param_id} to {param_value}")
        while True:
            message = self.conn.recv_match(type='PARAM_VALUE', blocking=True)
            if message and message.param_id == param_id:
                print(f"Parameter {param_id} set to {message.param_value}")
                break

    def arm_vehicle(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("Attempting to arm the vehicle...")
        timeout = time.time() + 10
        while time.time() < timeout:
            message = self.conn.recv_match(type='HEARTBEAT', blocking=True)
            if message and message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Vehicle is armed!")
                return True
        print("Failed to arm the vehicle.")
        return False

    def set_mode(self, mode):
        """Set the vehicle mode"""
        mode_id = self.conn.mode_mapping()[mode.upper()]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Setting mode to {mode}...")
        for _ in range(5):
            heartbeat = self.conn.recv_match(type='HEARTBEAT', blocking=True)
            if heartbeat and heartbeat.custom_mode == mode_id:
                print(f"Mode set to {mode}")
                return True
        print(f"Failed to set mode to {mode}")
        return False

    def set_rc_channels(self, rc_channels):
        """Set RC channels for skid steering control"""
        print(f"Setting RC channels: {rc_channels}")
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,
            self.conn.target_component,
            *rc_channels
        )

if __name__ == "__main__":
    boat_controller = BoatController('udpin:192.168.2.1:14550')
    
    boat_controller.set_parameter('ARMING_CHECK', 0)
    
    if not boat_controller.arm_vehicle():
        print("Exiting due to failed arming.")
        exit(1)

    if not boat_controller.set_mode('MANUAL'):
        print("Exiting due to failed mode change.")
        exit(1)

    # Define initial RC channel values (1500 is neutral for all channels)
    rc_channels = [1500] * 8

    # Activate left motor (e.g., channel 1) and right motor (e.g., channel 2)
    # Adjust these channel numbers and PWM values as per your setup
    
    #RIGHT
    #rc_channels[0] = 1600  # Increase throttle for left motor
    #rc_channels[2] = 1400  # Decrease throttle for right motor
    
    #LEFT
    rc_channels[0] = 1400  # Increase throttle for left motor
    rc_channels[2] = 1600  # Decrease throttle for right motor

    boat_controller.set_rc_channels(rc_channels)
    time.sleep(3)

    # Set all channels back to neutral (1500)
    rc_channels[0] = 1500
    rc_channels[2] = 1500

    boat_controller.set_rc_channels(rc_channels)
