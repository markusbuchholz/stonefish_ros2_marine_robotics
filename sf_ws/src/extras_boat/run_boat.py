from pymavlink import mavutil
import time

# Establish connection to the vehicle
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print("Waiting for Heartbeat")
master.wait_heartbeat()
print("Heartbeat received")

# Function to check and set mode
def set_mode(master, mode):
    mode_id = master.mode_mapping()[mode.upper()]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    # Check if mode has been set by listening to heartbeats
    for _ in range(5):
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
        current_mode = heartbeat.custom_mode
        if current_mode == mode_id:
            print(f"Mode set to {mode}")
            return True
    return False

# Arm the vehicle
def arm_vehicle(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    # Check for arming confirmation
    print("Arming vehicle...")
    for _ in range(5):
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Vehicle is armed!")
            return True
    print("Failed to arm vehicle.")
    return False

# Send RC control commands
def send_rc_command(master, channels):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7])

def main():
    if not arm_vehicle(master):
        return
    
    if not set_mode(master, 'MANUAL'):
        print("Failed to set mode to MANUAL.")
        return
    
    try:
        while True:
            cmd = input("Enter command (1: Forward, 2: Back, 3: Left, 4: Right, 0: Stop, q: Quit): ")
            if cmd == 'q':
                print("Exiting...")
                break
            elif cmd == '1':  # Forward
                send_rc_command(master, [1500, 1500, 1600, 1500, 1500, 1500, 1500, 1500])
            elif cmd == '2':  # Back
                send_rc_command(master, [1500, 1500, 1400, 1500, 1500, 1500, 1500, 1500])
            elif cmd == '3':  # Left
                send_rc_command(master, [1400, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
            elif cmd == '4':  # Right
                send_rc_command(master, [1600, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
            elif cmd == '0':  # Stop
                send_rc_command(master, [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
    except Exception as e:
        print(f"Error occurred: {str(e)}")

if __name__ == '__main__':
    main()
