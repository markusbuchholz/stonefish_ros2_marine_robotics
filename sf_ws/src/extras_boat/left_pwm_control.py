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

    def set_rc_channel(self, channel, pwm):
        """Set an individual RC channel"""
        rc_channel_values = [1500] * 8  # Neutral values for all channels
        rc_channel_values[channel - 1] = pwm  # Set specified channel
        print(f"Setting RC channel {channel} to PWM {pwm}")
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,
            self.conn.target_component,
            *rc_channel_values
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

    # Control only the left motor (e.g., channel 1)
    boat_controller.set_rc_channel(channel=1, pwm=1600)
    time.sleep(5)
    boat_controller.set_rc_channel(channel=1, pwm=1500)
