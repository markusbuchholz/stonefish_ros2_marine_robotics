#!/usr/bin/env python3

"""
Example program that:
1. Connects to the vehicle via MAVLink (UDP).
2. Arms the vehicle and sets MANUAL mode.
3. Prompts the user for RC override commands.
4. Continuously prints SERVO_OUTPUT_RAW messages in the background (every 0.1s),
   but in a simpler format: "1: value, 2: value, 3: value, etc."
"""

import time
import threading
from pymavlink import mavutil

# Establish connection to the vehicle
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print("Waiting for Heartbeat")
master.wait_heartbeat()
print("Heartbeat received")

# Function to check and set mode
def set_mode(master, mode):
    """
    Sets the flight mode by name (e.g., 'MANUAL', 'STABILIZE', etc.)
    Returns True if successful, False otherwise.
    """
    mode = mode.upper()
    mode_mapping = master.mode_mapping()
    if mode not in mode_mapping:
        print(f"Mode '{mode}' is not available on this vehicle.")
        return False

    mode_id = mode_mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Check if mode has been set by listening to heartbeats
    for _ in range(5):
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat is None:
            continue
        current_mode = heartbeat.custom_mode
        if current_mode == mode_id:
            print(f"Mode set to {mode}")
            return True
    return False

# Arm the vehicle
def arm_vehicle(master):
    """
    Arms the vehicle and waits for confirmation via HEARTBEAT message.
    Returns True if armed, False otherwise.
    """
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    # Check for arming confirmation
    print("Arming vehicle...")
    for _ in range(5):
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat is not None:
            if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Vehicle is armed!")
                return True
    print("Failed to arm vehicle.")
    return False

# Send RC control commands
def send_rc_command(master, channels):
    """
    Overrides 8 RC channels with the values in 'channels' list.
    Values must be integers. 1500 is neutral/stop, 1000/2000 extremes, etc.
    """
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7]
    )

def move_robot(master, thrust_left, thrust_right):
    """
    Example function to demonstrate custom movement logic by combining thrust values.
    This sets channel 1 and 3 in a simplified "differential thrust" manner.
    """
    # Convert thrust values to (example) PWM values
    ch1_pwm = 1500 + (thrust_right - thrust_left)  
    ch3_pwm = 1500 + ((thrust_left + thrust_right) // 2)

    # Clamp PWM values to valid range (1100 to 1900 for safety)
    ch1_pwm = max(1100, min(1900, ch1_pwm))
    ch3_pwm = max(1100, min(1900, ch3_pwm))
    print(f"move_robot -> ch1_pwm: {ch1_pwm}, ch3_pwm: {ch3_pwm}")

    # Send RC overrides on channels 1, 3, keep others at 1500
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        ch1_pwm, 1500, ch3_pwm, 1500,
        1500, 1500, 1500, 1500
    )

def print_servo_outputs(master):
    """
    Continuously reads MAVLink messages and prints only SERVO_OUTPUT_RAW
    in "1: val, 2: val, ..." format every 0.1s in a background thread.
    """
    while True:
        msg = master.recv_match()
        if msg:
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                servo_dict = msg.to_dict()
                # Construct a formatted string for servo1_raw through servo16_raw
                # Example: "1: 1500, 2: 1400, 3: 1900, ..."
                servo_output_str = []
                for i in range(1, 17):
                    servo_output_str.append(f"{i}: {servo_dict.get(f'servo{i}_raw', 0)}")

                # Print the final line (or do other logging as needed)
                print("[SERVO_OUTPUT_RAW]", ", ".join(servo_output_str))
        time.sleep(0.1)

def main():
    # Start a background thread that prints SERVO_OUTPUT_RAW
    t = threading.Thread(target=print_servo_outputs, args=(master,), daemon=True)
    t.start()

    # Arm the vehicle
    if not arm_vehicle(master):
        return
    
    # Set flight mode to MANUAL
    if not set_mode(master, 'MANUAL'):
        print("Failed to set mode to MANUAL.")
        return
    
    try:
        # Main loop for user input
        while True:
            cmd = input("Enter command (1: Forward, 2: Back, 3: Left, 4: Right, 0: Stop, 9:move_robot, q: Quit): ")
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
            elif cmd == '9':  # Example usage of move_robot()
                move_robot(master, thrust_left=200, thrust_right=600)
            else:
                print("Unknown command. Try again.")
    except Exception as e:
        print(f"Error occurred: {str(e)}")

if __name__ == '__main__':
    main()
