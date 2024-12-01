#!/usr/bin/env python
from pymavlink import mavutil
from time import sleep

# Function to create a connection to the autopilot
def create_connection():
    return mavutil.mavlink_connection('udpin:192.168.2.1:14550')

def send_rc_channels(master, channels, duration):
    # Prepare the channels list, ensuring it contains exactly 8 values
    channels = (channels + [0]*8)[:8]  # Extend and slice the list to ensure exactly 8 elements
    
    # Send RC channels override
    master.mav.rc_channels_override_send(
        master.target_system,       # target system
        master.target_component,    # target component
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7]
    )
    sleep(duration)
    
    # Reset channels to 1500 for stopping (neutral position)
    neutral_channels = [1500]*8
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *neutral_channels
    )



def move_forward(master):
    send_rc_channels(master, [1550, 0, 1500, 1500, 1500, 1500, 1500, 1500], 1.0)
    send_rc_channels(master, [1540, 0, 1500, 1500, 1500, 1500, 1500, 1500], 4.0)

def move_left(master):
    send_rc_channels(master, [1540, 0, 1450, 1500, 1500, 1500, 1500, 1500], 3.0)

def move_right(master):
    send_rc_channels(master, [1540, 0, 1540, 1500, 1500, 1500, 1500, 1500], 3.0)


def move_back(master):
    send_rc_channels(master, [1450, 0, 1500, 1500, 1500, 1500, 1500, 1500], 4.0)

def stop_movement(master):
    send_rc_channels(master, [1500, 0, 1500, 1500, 1500, 1500, 1500, 1500], 5.0)



if __name__ == '__main__':
    master = create_connection()
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
    try:
        while True:
            cmd = input("Enter command (1: Forward, 2: Left, 3: Right, 4: Back, 0: Stop, q: Quit): ")
            if cmd == 'q':
                print("Exiting...")
                break
            elif cmd == '1':
                move_forward(master)
            elif cmd == '2':
                move_left(master)
            elif cmd == '3':
                move_right(master)
            elif cmd == '4':
                move_back(master)
            elif cmd == '0':
                stop_movement(master)
            else:
                print("Invalid command:", cmd)
    except Exception as e:
        print("Error:", e)
