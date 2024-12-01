
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
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)  # 4 


while True:
   
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack_msg is not None:
        print(f"Received ACK: {ack_msg.command} with result: {ack_msg.result}")
        break


    time.sleep(0.1)

print("Vehicle is now in GUIDED mode.")

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qw, qx, qy, qz]


def set_attitude(yaw_angle, yaw_rate=0.0, use_yaw_rate=False):
 
    q = euler_to_quaternion(0, 0, np.radians(yaw_angle))

   
    type_mask = 0b00000111 if not use_yaw_rate else 0b00000011

   
    master.mav.set_attitude_target_send(
        time_boot_ms=0,
        target_system=master.target_system,
        target_component=master.target_component,
        type_mask=type_mask,
        q=q,
        body_roll_rate=0,
        body_pitch_rate=0,
        body_yaw_rate=yaw_rate if use_yaw_rate else 0,
        thrust=0.5
    )


set_attitude(195)


time.sleep(5)