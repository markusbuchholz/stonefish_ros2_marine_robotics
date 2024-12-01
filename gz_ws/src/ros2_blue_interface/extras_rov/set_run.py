from pymavlink import mavutil
import time


master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


print("Waiting for vehicle to connect...")
master.wait_heartbeat()
print("Heartbeat from vehicle (system ID: %d, component ID: %d)" % (master.target_system, master.target_component))


motor_id = 1 
pwm_value = 1500  


def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )
#rc_channels_override_send
print("arm............................")
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


# command servo_1 to go from min to max in steps of 50us, over 2 seconds
for us in range(1100, 1900, 50):
    set_servo_pwm(1, us)
    time.sleep(0.125)

print("Motor command sent. Check your vehicle.")

#

time.sleep(3)

print ("disarm.............................")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
master.motors_disarmed_wait()