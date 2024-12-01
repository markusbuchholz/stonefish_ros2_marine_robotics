from pymavlink import mavutil
import time

print("settting light on on vehicle") 

# Create the connection
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()



# Arm
# master.arducopter_arm() or:
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

# # wait until arming confirmed (can manually check with master.motors_armed())
# print("Waiting for the vehicle to arm")
# master.motors_armed_wait()
# print('Armed!')

# time.sleep (1)

# Disarm
# master.arducopter_disarm() or:
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
# master.motors_disarmed_wait()
# PWM values for lights ON and OFF
def set_rc_channel_pwm(channel_id, pwm=1500):
    print("called")
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


set_rc_channel_pwm(9, 1600)
time.sleep(5)

