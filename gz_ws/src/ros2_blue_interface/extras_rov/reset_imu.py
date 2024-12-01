from pymavlink import mavutil
import time

def print_imu_data(message):
    if message is not None and message.get_type() == 'HIGHRES_IMU':
        print(f"Time: {message.time_usec} us")
        print(f"Accelerations: x={message.xacc} y={message.yacc} z={message.zacc} (m/s^2)")
        print(f"Angular Velocities: x={message.xgyro} y={message.ygyro} z={message.zgyro} (rad/s)")
        print(f"Magnetic Fields: x={message.xmag} y={message.ymag} z={message.zmag} (Gauss)")
        print(f"Pressure: {message.abs_pressure} hPa")
        print(f"Temperature: {message.temperature} C")
        print("---------------------------------------------------")

print("Arming / Disarming")
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat()
print("Heartbeat received")

print("......status BEFORE reset......")
message = master.recv_match(type='HIGHRES_IMU', blocking=True, timeout=3)
print_imu_data(message)

# Send a command to reset IMU data estimates
print("Sending reset command...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0,  # confirmation
    1, 0, 0, 0, 0, 0, 0  # parameters: 1 to reboot autopilot, others are zero
)

time.sleep(10)  # Wait for the system to reboot

print("....status AFTER reset......")
message = master.recv_match(type='HIGHRES_IMU', blocking=True, timeout=3)
print_imu_data(message)
