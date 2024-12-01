from pymavlink import mavutil
import time

class BoatController:
    def __init__(self, connection_string):
        # Create the connection
        self.conn = mavutil.mavlink_connection(connection_string)
        # Wait for a heartbeat before sending commands
        self.conn.wait_heartbeat()
    
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

    def print_status(self, description):
        print(f"Current status {description}:")
        message = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if message:
            print(message.to_dict())
        else:
            print("No heartbeat message received.")

if __name__ == "__main__":
    boat_controller = BoatController('udpin:192.168.2.1:14550')
    boat_controller.arm_vehicle()
    boat_controller.set_guided_mode()
