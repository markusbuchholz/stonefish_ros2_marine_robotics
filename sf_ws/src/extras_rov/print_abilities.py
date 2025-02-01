import time
from pymavlink import mavutil

# Define sensor bit mapping for SYS_STATUS
# The key is the bit number (0 = least-significant bit) and the value is a tuple:
#   (short_name, human_readable_name)
SENSOR_DEFINITIONS = {
    0: ("3D_GYRO", "3D Gyroscope"),
    1: ("3D_ACCEL", "3D Accelerometer"),
    2: ("3D_MAG", "3D Magnetometer"),
    3: ("ABS_PRESSURE", "Absolute Pressure Sensor"),
    4: ("DIFF_PRESSURE", "Differential Pressure Sensor"),
    5: ("GPS", "GPS Receiver"),
    6: ("OPTICAL_FLOW", "Optical Flow Sensor"),
    7: ("VISION_POSITION", "Vision Position Sensor"),
    8: ("LASER_POSITION", "Laser Position Sensor"),
    9: ("EXTERNAL_GROUND_TRUTH", "External Ground Truth"),
    10: ("ANGULAR_RATE_CONTROL", "Angular Rate Controller"),
    11: ("ATTITUDE_STABILIZATION", "Attitude Stabilization"),
    12: ("YAW_POSITION", "Yaw Position"),
    13: ("Z_ALTITUDE", "Z Altitude"),
    # Add further sensor definitions if your firmware supports more bits
}

def decode_sensor_status(bitmask, sensor_definitions):
    """
    Given a bitmask (integer) and a sensor definitions dictionary,
    returns a dictionary with sensor names as keys and True/False indicating status.
    """
    status = {}
    for bit, (short_name, full_name) in sensor_definitions.items():
        status[full_name] = bool(bitmask & (1 << bit))
    return status

def print_sensor_status(sensors_present, sensors_enabled, sensors_health):
    """
    For each sensor defined in SENSOR_DEFINITIONS, print its status.
    """
    present = decode_sensor_status(sensors_present, SENSOR_DEFINITIONS)
    enabled = decode_sensor_status(sensors_enabled, SENSOR_DEFINITIONS)
    healthy = decode_sensor_status(sensors_health, SENSOR_DEFINITIONS)

    print("\n--- Sensor Status ---")
    for sensor in SENSOR_DEFINITIONS.values():
        full_name = sensor[1]
        # Get boolean values for present, enabled, healthy; default to False if not set.
        is_present = present.get(full_name, False)
        is_enabled = enabled.get(full_name, False)
        is_healthy = healthy.get(full_name, False)
        print(f"{full_name:30s} | Present: {'Yes' if is_present else 'No':3s} | "
              f"Enabled: {'Yes' if is_enabled else 'No':3s} | Healthy: {'Yes' if is_healthy else 'No':3s}")

def print_battery_info(data):
    voltage = data.get("voltage_battery", None)
    current = data.get("current_battery", None)
    battery_remaining = data.get("battery_remaining", None)

    print("\n--- Battery Information ---")
    print("Battery Voltage (mV):", voltage if voltage is not None else "N/A")
    if current is not None:
        # The unit is 10*mA. Multiply by 10 for mA.
        print("Battery Current (mA):", current * 10)
    else:
        print("Battery Current: N/A")
    print("Battery Remaining (%):", battery_remaining if battery_remaining is not None else "N/A")

def main():
    # Create the connection to the vehicle (using UDP input on port 14550)
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()
    print("Heartbeat received from system")

    while True:
        try:
            msg = master.recv_match(blocking=True)
            if msg is None:
                continue

            msg_type = msg.get_type()
            if msg_type == "SYS_STATUS":
                data = msg.to_dict()

                # Print battery info
                print_battery_info(data)

                # Get sensor bitmask fields (they come as integers)
                sensors_present = data.get("onboard_control_sensors_present", 0)
                sensors_enabled = data.get("onboard_control_sensors_enabled", 0)
                sensors_health  = data.get("onboard_control_sensors_health", 0)

                # Print raw bitmask values (optional)
                print("\nRaw Bitmasks:")
                print("Sensors Present: 0b{:032b}".format(sensors_present))
                print("Sensors Enabled: 0b{:032b}".format(sensors_enabled))
                print("Sensors Health:  0b{:032b}".format(sensors_health))

                # Print human-readable sensor status
                print_sensor_status(sensors_present, sensors_enabled, sensors_health)

        except Exception as e:
            print("Error:", e)

        # Short delay to avoid overwhelming the output
        time.sleep(0.1)

if __name__ == '__main__':
    main()
