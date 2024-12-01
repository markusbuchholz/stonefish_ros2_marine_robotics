from pymavlink import mavutil
import time

def set_ekf_type_and_reboot(master, ekf_type):
    param_name = 'AHRS_EKF_TYPE'
    # Set the EKF type
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode(),
        float(ekf_type),
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
    )
    print(f"Setting EKF type to {ekf_type}")
    time.sleep(2)  # Allow time for the parameter to be set

    # Reboot the vehicle to apply changes
    print("Rebooting vehicle to apply changes...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,  # Confirmation
        1,  # Reboot the autopilot
        0, 0, 0, 0, 0, 0
    )

    # Wait for the vehicle to disconnect and then attempt to reconnect
    time.sleep(10)  # Adjust based on your vehicle's typical reboot time
    print("Reconnecting to vehicle...")
    master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
    master.wait_heartbeat()
    print("Reconnected to vehicle after reboot.")
    # Fetch and print the set EKF type to verify
    master.param_fetch_all()
    time.sleep(2)  # Allow time for all parameters to be read
    current_type = master.param_fetch_one(param_name)
    print(f"Current EKF type after reboot: {current_type}")

# Setup MAVLink connection
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat()
print("Connected to vehicle and heartbeat received.")

# Set the EKF type and reboot
set_ekf_type_and_reboot(master, 2)  # Set EKF2 or EKF3 as needed
