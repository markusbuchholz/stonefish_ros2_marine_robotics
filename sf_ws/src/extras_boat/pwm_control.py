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
    
def move_robot(master, thrust_left, thrust_right):
    """Control the robot's movement by setting the RC channels"""
    # Convert thrust values to PWM values for channels 1 and 3
    ch1_pwm = 1500 + (thrust_right - thrust_left)  # Thrust control for left/right
    ch3_pwm = 1500 + ((thrust_left + thrust_right) // 2)  # Forward/backward control

    # Clamp PWM values to valid range (1000 to 2000)
    ch1_pwm = max(1100, min(1900, ch1_pwm))
    ch3_pwm = max(1100, min(1900, ch3_pwm))
    print (ch1_pwm, ", ", ch3_pwm)

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        ch1_pwm, 1500, ch3_pwm, 1500,
        1500, 1500, 1500, 1500)

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
            elif cmd == '9':
                move_robot(master,thrust_left=200, thrust_right=200)
                
    
    except Exception as e:
        print(f"Error occurred: {str(e)}")

if __name__ == '__main__':
    main()
