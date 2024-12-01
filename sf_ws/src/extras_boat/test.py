import time
from pymavlink import mavutil

# Constants
MOTOR_TEST_COMMAND = mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST
MOTOR_TEST_THROTTLE_TYPE = 1  # 1 for throttle as percentage
TEST_DURATION = 2  # Test duration in seconds
MANUAL_MODE = 19  # For ArduSub manual mode
ARMING_CHECK_OVERRIDE = 21196
MOTOR_TEST_ORDER = 0  # 0 for testing individual motors

# Connect to the vehicle
def connect_to_vehicle(connection_string):
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = mavutil.mavlink_connection(connection_string)
    vehicle.wait_heartbeat()
    print("Heartbeat received from vehicle.")
    return vehicle

# Set vehicle mode
def set_vehicle_mode(vehicle, mode):
    print(f"Setting vehicle mode to {mode}...")
    vehicle.set_mode(mode)

    # Confirm mode change
    while True:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            print(f"Received heartbeat: custom_mode={msg.custom_mode}, base_mode={msg.base_mode}")
            if msg.custom_mode == mode:
                print(f"Vehicle mode set to {mode}.")
                break
        time.sleep(1)

# Arm the vehicle
def arm_vehicle(vehicle):
    print("Arming vehicle...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, ARMING_CHECK_OVERRIDE, 0, 0, 0, 0, 0
    )

    # Confirm arming
    while True:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            print(f"Received heartbeat: custom_mode={msg.custom_mode}, base_mode={msg.base_mode}")
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Vehicle armed.")
                break
        time.sleep(1)

# Disarm the vehicle
def disarm_vehicle(vehicle):
    print("Disarming vehicle...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    # Confirm disarming
    while True:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            print(f"Received heartbeat: custom_mode={msg.custom_mode}, base_mode={msg.base_mode}")
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Vehicle disarmed.")
                break
        time.sleep(1)

# Test multiple motors
def test_motors(vehicle, motor_ids, thrust_percent):
    print(f"Testing motors {motor_ids} with {thrust_percent}% thrust for {TEST_DURATION} seconds.")
    for motor_id in motor_ids:
        print(f"Sending motor test command for motor {motor_id}")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            MOTOR_TEST_COMMAND,
            0,
            motor_id,          # param1: motor id
            MOTOR_TEST_THROTTLE_TYPE,  # param2: throttle type (1 for percentage)
            thrust_percent,    # param3: throttle as percentage
            TEST_DURATION,     # param4: test duration
            MOTOR_TEST_ORDER,  # param5: motor order (0 for specific motor)
            0,
            0
        )
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
        if ack and ack.command == MOTOR_TEST_COMMAND:
            print(f"Motor {motor_id} test command acknowledged.")
        else:
            print(f"Failed to receive acknowledgement for motor {motor_id} test command.")
        time.sleep(0.1)  # Slight delay to ensure the command is processed

    # Log heartbeat during motor test
    end_time = time.time() + TEST_DURATION
    while time.time() < end_time:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            print(f"Heartbeat during test: custom_mode={msg.custom_mode}, base_mode={msg.base_mode}")
        time.sleep(0.5)

# Main function
def main():
    connection_string = 'udp:192.168.2.1:14550'  # Adjust this as needed
    vehicle = connect_to_vehicle(connection_string)

    try:
        set_vehicle_mode(vehicle, MANUAL_MODE)
        arm_vehicle(vehicle)

        motor_ids = [9]  # List of motor IDs to test
        thrust_percent = 70  # Desired thrust percentage (70% to ensure movement)

        test_motors(vehicle, motor_ids, thrust_percent)

        # Allow motors to run for test duration
        time.sleep(TEST_DURATION)

    finally:
        disarm_vehicle(vehicle)

if __name__ == "__main__":
    main()
