# Import mavutil
from pymavlink import mavutil
import time

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


def set_rc_channels_pwm(vals):

    rc_channel_values = [int(val) for val in vals[:9]]
    #print("values:", rc_channel_values)

    while len(rc_channel_values) < 9:
        rc_channel_values.append(0)

    master.mav.rc_channels_override_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        rc_channel_values[0],
        rc_channel_values[1],
        rc_channel_values[2],
        rc_channel_values[3],
        rc_channel_values[4],
        rc_channel_values[5],
        rc_channel_values[6],
        rc_channel_values[7],
        rc_channel_values[8],
    )

pwm_values = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1900]
set_rc_channels_pwm(pwm_values)

time.sleep (5)

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
master.motors_disarmed_wait()