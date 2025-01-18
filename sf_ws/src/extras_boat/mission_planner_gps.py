# Markus Buchholz
# Mission planner for BlueBoat (GPS)

from pymavlink import mavutil
import time
import math

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550') 

master.wait_heartbeat()

def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    print("Attempting to arm the vehicle...")
    timeout = time.time() + 10  # 10 second timeout for arming
    armed = False

    while not armed and time.time() < timeout:
        message = master.recv_match(type='HEARTBEAT', blocking=True)
        if message:
            print(message.to_dict())
            armed = message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print('Vehicle is armed!')
                break

    if not armed:
        print('Failed to arm the vehicle. Check pre-arm conditions and messages.')

def set_guided_mode():
    mode = 'GUIDED'
    if mode not in master.mode_mapping():
        print(f"Mode {mode} not found in mode mapping. Exiting...")
        return
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Mode set to GUIDED.")

def get_initial_position():
    print("Waiting for GPS position...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            initial_lat = msg.lat / 1e7
            initial_lon = msg.lon / 1e7
            print(f"Initial position: lat={initial_lat}, lon={initial_lon}")
            return initial_lat, initial_lon

def send_target_position(lat, lon):
    # Send the target position to the vehicle
    master.mav.set_position_target_global_int_send(
        0,                         # time_boot_ms
        master.target_system,      # target_system
        master.target_component,   # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000,        # type_mask (only positions enabled)
        int(lat * 1e7),            # lat_int - X Position in WGS84 frame in 1e7 * degrees
        int(lon * 1e7),            # lon_int - Y Position in WGS84 frame in 1e7 * degrees
        0,                         # alt
        0, 0, 0,                   # X, Y, Z velocity in m/s (not used)
        0, 0, 0,                   # afx, afy, afz acceleration (not used)
        0, 0)                      # yaw, yaw rate (not used)

def follow_target_positions(initial_lat, initial_lon):
    # mission sequence to follow (in meters) for Boat
    positions = [(0, 0), (5, 5), (0, 0), (10, -10), (-5, 5), (0, 0), (30, 30), (0, 0)]  # Positions in meters

    for idx, (x, y) in enumerate(positions):
        
        lat = initial_lat + (y / 111320)  # Latitude degree per meter
        lon = initial_lon + (x / (40075000 * (1 / 360) * math.cos(math.radians(initial_lat))))  # Longitude degree per meter
        
        send_target_position(lat, lon)
        print(f"Sent target position {idx + 1}/{len(positions)}: lat={lat}, lon={lon}")

        # Wait for the rover to reach the position with a timeout
        start_time = time.time()
        timeout = 60  # 60 seconds timeout
        while time.time() - start_time < timeout:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7

                # Check if the BlueBoat has reached the target position
                if abs(current_lat - lat) < 0.00005 and abs(current_lon - lon) < 0.00005:  # Increased tolerance
                    print(f"Reached target position {idx + 1}/{len(positions)}: lat={lat}, lon={lon}")
                    break
        else:
            print(f"Timeout reached while waiting for target position {idx + 1}/{len(positions)}")

        time.sleep(2)  

if __name__ == "__main__":
    arm_vehicle()
    set_guided_mode()
    initial_lat, initial_lon = get_initial_position()
    follow_target_positions(initial_lat, initial_lon)
