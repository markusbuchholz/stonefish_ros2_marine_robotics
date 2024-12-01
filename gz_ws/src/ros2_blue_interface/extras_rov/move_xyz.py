
from pymavlink import mavutil
import math
import time
import numpy as np

# Import mavutil
from pymavlink import mavutil
import time

print("Arming / Disarming") 

# Create the connection
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

# Arm
# master.arducopter_arm() or:
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

# Request MAVLink parameters to confirm connection
master.mav.param_request_list_send(master.target_system, master.target_component)

# Fetch all parameters (might take a while depending on the system)
while True:
    message = master.recv_match(type='PARAM_VALUE', blocking=True)
    print(f"Parameter: {message.param_id}, Value: {message.param_value}")
    if message.param_index + 1 == message.param_count:  # Exit loop once all parameters are received
        break

print("Connected to vehicle.")

# Set vehicle to GUIDED mode
# master.mav.set_mode_send(
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     4)  # 4 


# while True:
   
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
#     if ack_msg is not None:
#         print(f"Received ACK: {ack_msg.command} with result: {ack_msg.result}")
#         break


#     time.sleep(0.1)

# print("Vehicle is now in GUIDED mode.")

def move_to_position(x, y, z, duration):
    # Define the message parameters
    time_boot_ms = 0  # Timestamp (milliseconds, not used)
    target_system = master.target_system
    target_component = master.target_component
    coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED  # Use local North-East-Down frame
    type_mask = 0b0000110111111000  # Position only

    # Set command to move the vehicle to a specified position
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        target_system,
        target_component,
        coordinate_frame,
        type_mask,
        x,  # North position in meters
        -y,  # East position in meters
        -z,  # 
        0,  # X velocity (not used)
        0,  # Y velocity (not used)
        0,  # Z velocity (not used)
        0,  # X acceleration (not used)
        0,  # Y acceleration (not used)
        0,  # Z acceleration (not used)
        0,  # Yaw angle (not used)
        0   # Yaw rate (not used)
    )

    # Wait for the duration of the move to allow it to complete
    #time.sleep(duration)

# Move the robot 1 meter in the X direction over a period of 5 seconds
move_to_position(0, 0, -0.5, 3)