from pymavlink import mavutil
import time

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

# Check if mode change was successful
while True:
    # Wait for ACK from the vehicle
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack_msg is not None:
        print(f"Received ACK: {ack_msg.command} with result: {ack_msg.result}")
        break

    # Safety timeout to prevent infinite loop
    time.sleep(0.1)

print("Vehicle is now in GUIDED mode.")
