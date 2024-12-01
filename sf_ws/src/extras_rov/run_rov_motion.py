from pymavlink import mavutil
from time import sleep, time
import numpy as np

def create_connection():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def set_vehicle_mode(vehicle, mode):
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.set_mode(mode_id)
    print(f"Vehicle mode set to {mode}")

def arm_vehicle(vehicle):
    vehicle.arducopter_arm()
    vehicle.motors_armed_wait()
    print("Vehicle armed")

def disarm_vehicle(vehicle):
    vehicle.arducopter_disarm()
    vehicle.motors_disarmed_wait()
    print("Vehicle disarmed")

def set_servo_function(vehicle, servo_num, function):
    param_name = f'SERVO{servo_num}_FUNCTION'
    vehicle.mav.param_set_send(
        vehicle.target_system,
        vehicle.target_component,
        param_name.encode('utf-8'),
        function,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )

def enable_passthrough_mode(vehicle, num_thrusters):
    for i in range(1, num_thrusters + 1):
        set_servo_function(vehicle, i, 1)
        sleep(0.1)

def disable_passthrough_mode(vehicle, num_thrusters, backup_params):
    for i in range(1, num_thrusters + 1):
        set_servo_function(vehicle, i, backup_params[i-1])
        sleep(0.1)

def backup_thruster_params(vehicle, num_thrusters):
    backup_params = []
    for i in range(1, num_thrusters + 1):
        param_name = f'SERVO{i}_FUNCTION'
        vehicle.mav.param_request_read_send(
            vehicle.target_system,
            vehicle.target_component,
            param_name.encode('utf-8'),
            -1
        )
        message = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        backup_params.append(int(message.param_value))
    return backup_params

def fetch_current_state_local(vehicle):
    print('Fetching current state...')
    default_position = {'x': 0, 'y': 0, 'z': 0}
    default_orientation = {'yaw': 0}

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        0, 0, 0, 0, 0, 0, 0
    )
    position_message = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    if position_message:
        print(f"Position received: x={position_message.x}, y={position_message.y}, z={position_message.z}")
        position = {'x': position_message.x, 'y': position_message.y, 'z': position_message.z}
    else:
        print("Failed to receive position message.")
        position = default_position

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        0, 0, 0, 0, 0, 0, 0
    )
    attitude_message = vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    if attitude_message:
        print(f"Attitude received: yaw={attitude_message.yaw}")
        orientation = {'yaw': attitude_message.yaw}
    else:
        print("Failed to receive attitude message.")
        orientation = default_orientation

    print("--------------------------------------")
    print("x: ", position['x'], 'y:', position['y'], 'z:', position['z'], 'yaw:', orientation['yaw'])
    return {
        'x': position['x'],
        'y': position['y'],
        'z': position['z'],
        'yaw': orientation['yaw']
    }

def wait_for_valid_position(vehicle, max_attempts=200):
    for attempt in range(max_attempts):
        current_state = fetch_current_state_local(vehicle)
        if current_state['x'] != 0 or current_state['y'] != 0 or current_state['z'] != 0:
            return current_state
        print(f"Invalid position data, retrying... ({attempt + 1}/{max_attempts})")
        sleep(1)
    raise TimeoutError("Failed to get a valid position")

def set_guided_mode_hold_position_local(vehicle):
    print("Setting GUIDED mode and holding position...")
    current_state = wait_for_valid_position(vehicle)

    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )

    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        current_state['x'],
        current_state['y'],
        current_state['z'],
        0, 0, 0,
        0, 0, 0,
        current_state['yaw'],
        0
    )
    print(f"Holding position at x={current_state['x']}, y={current_state['y']}, z={current_state['z']}, yaw={current_state['yaw']}")
    return current_state

def send_position_request(vehicle, x, y, z):
    type_mask = 0b0000111111111000  # Only positions
    coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    vehicle.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        vehicle.target_system,  # Target system
        vehicle.target_component,  # Target component
        coordinate_frame,  # Coordinate frame
        type_mask,  # Type mask
        x, y, z,  # Positions in NED frame
        0, 0, 0,  # Velocities (not used)
        0, 0, 0,  # Accelerations (not used)
        0, 0  # Yaw and yaw rate (not used)
    )
    print(f"Position request sent: x={x}, y={y}, z={z}")

def wait_until_position_reached(vehicle, target_position, margin=0.2, timeout=60):
    """
    Wait until the vehicle reaches the target position within a margin.
    """
    start_time = time()
    while time() - start_time < timeout:
        current_state = fetch_current_state_local(vehicle)
        dx = current_state['x'] - target_position['x']
        dy = current_state['y'] - target_position['y']
        dz = current_state['z'] - target_position['z']
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        print(f"Current distance to target: {distance:.2f} meters")
        if distance <= margin:
            print("Target position reached within margin.")
            return current_state
        sleep(1)
    print("Timeout exceeded while waiting for target position.")
    return current_state  # Return last known state even if timeout

def execute_movements(vehicle, target_movements, margin=0.2, timeout=60):
    """
    Execute a series of target movements sequentially.
    
    Parameters:
        vehicle: The MAVLink vehicle connection.
        target_movements: List of dictionaries with 'delta_x', 'delta_y', 'delta_z'.
        margin: Distance margin to consider the target as reached.
        timeout: Maximum time to wait for each target.
    """
    current_state = fetch_current_state_local(vehicle)
    for idx, movement in enumerate(target_movements, start=1):
        print(f"  -------- Movement {idx} --------")
        delta_x = movement.get('delta_x', 0.0)
        delta_y = movement.get('delta_y', 0.0)
        delta_z = movement.get('delta_z', 0.0)

        # Calculate new position in NED frame
        new_x = current_state['x'] + delta_x
        new_y = current_state['y'] + delta_y
        new_z = current_state['z'] + delta_z

        # Send a position request to move the vehicle to the new position
        send_position_request(vehicle, new_x, new_y, new_z)

        # Wait until position is reached
        target_position = {'x': new_x, 'y': new_y, 'z': new_z}
        current_state = wait_until_position_reached(vehicle, target_position, margin, timeout)

def main():
    connection_string = 'udp:127.0.0.1:14550'  # Replace with your connection string
    vehicle = create_connection()
    vehicle.wait_heartbeat()
    print("Heartbeat received")

    num_thrusters = 8  # Number of thrusters to control

    try:
        backup_params = backup_thruster_params(vehicle, num_thrusters)
        print("Thruster parameters backed up")

        # Uncomment the following lines if you need to enable passthrough mode
        # enable_passthrough_mode(vehicle, num_thrusters)
        # print("RC Passthrough mode enabled")

        current_state = set_guided_mode_hold_position_local(vehicle)

        arm_vehicle(vehicle)

        # Define target movements as a list of dictionaries
        target_movements = [
            {
                'delta_x': 0.0,   # Move 0 meters North
                'delta_y': 0.0,   # No movement East
                'delta_z': 1.5    # Move down 1.5 meters (positive in NED frame)
            },
            {
                'delta_x': 0.0,   # Move 0 meters North
                'delta_y': 0.0,   # No movement East
                'delta_z': -1.0   # Move up 1.0 meters
            },
            {
                'delta_x': 1.0,   # Move 1 meter South
                'delta_y': 1.0,   # Move 1 meter East
                'delta_z': 0.0    # No vertical movement
            },
            # Add more target movements here as needed
        ]

        # Execute all target movements
        execute_movements(vehicle, target_movements)

        disarm_vehicle(vehicle)
        # Uncomment the following line if you enabled passthrough mode
        # disable_passthrough_mode(vehicle, num_thrusters, backup_params)
        print("Mission completed.")
    except Exception as e:
        print("Error:", e)
        disarm_vehicle(vehicle)
        # Uncomment the following line if you enabled passthrough mode
        # disable_passthrough_mode(vehicle, num_thrusters, backup_params)
        print("Vehicle disarmed due to error")

if __name__ == "__main__":
    main()
