from pymavlink import mavutil
import math
import time

# Constants
RADIUS_EARTH = 6378137.0  # Radius of Earth in meters

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance between two points on the Earth's surface."""
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return RADIUS_EARTH * c

def create_connection():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def get_gps_position(vehicle):
    """Get the current GPS position from the vehicle."""
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        0, 0, 0, 0, 0, 0, 0
    )
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if message:
        lat = message.lat / 1e7
        lon = message.lon / 1e7
        alt = message.alt / 1e3  # Convert from mm to meters
        return lat, lon, alt
    else:
        raise TimeoutError("Failed to get GPS position")

def convert_gps_to_xy(lat1, lon1, lat2, lon2):
    """Convert GPS coordinates to local XY coordinates."""
    x = haversine_distance(lat1, lon1, lat1, lon2)
    y = haversine_distance(lat1, lon1, lat2, lon1)
    # Adjust signs based on relative position
    if lon2 < lon1:
        x = -x
    if lat2 < lat1:
        y = -y
    return x, y

def main():
    vehicle = create_connection()
    vehicle.wait_heartbeat()
    print("Heartbeat received")

    # Get initial GPS position
    lat0, lon0, alt0 = get_gps_position(vehicle)
    print(f"Initial GPS position: lat={lat0}, lon={lon0}, alt={alt0}")

    try:
        while True:
            # Get current GPS position
            lat, lon, alt = get_gps_position(vehicle)
            print(f"Current GPS position: lat={lat}, lon={lon}, alt={alt}")

            # Convert to local XY coordinates
            x, y = convert_gps_to_xy(lat0, lon0, lat, lon)
            print(f"Position relative to start: x={x:.2f} m, y={y:.2f} m")

            # Add a small delay to avoid spamming the output
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program terminated by user")

if __name__ == "__main__":
    main()
