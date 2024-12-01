import time
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2, degrees

# Constants
EARTH_RADIUS = 6371000  # in meters (approximate Earth radius)
REFERENCE_LAT = 55.9954141  # Reference latitude in degrees
REFERENCE_LON = -3.3010227  # Reference longitude in degrees

# Function to convert lat/lon to meters using the Haversine formula for small distances
def latlon_to_meters(lat, lon, ref_lat, ref_lon):
    # Convert degrees to radians
    lat_rad = radians(lat)
    lon_rad = radians(lon)
    ref_lat_rad = radians(ref_lat)
    ref_lon_rad = radians(ref_lon)

    # Compute differences
    dlat = lat_rad - ref_lat_rad
    dlon = lon_rad - ref_lon_rad

    # Calculate distance in meters
    a = sin(dlat / 2) ** 2 + cos(ref_lat_rad) * cos(lat_rad) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Distance in meters
    distance = EARTH_RADIUS * c

    # Calculate bearing to find x (East) and y (North) components
    y = sin(dlon) * cos(lat_rad)
    x = cos(ref_lat_rad) * sin(lat_rad) - sin(ref_lat_rad) * cos(lat_rad) * cos(dlon)
    bearing = atan2(y, x)

    # X (East) and Y (North) in meters
    x_meters = distance * cos(bearing)
    y_meters = distance * sin(bearing)

    return x_meters, y_meters

# Create the connection to MAVLink
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for a heartbeat to ensure connection
master.wait_heartbeat()

print("Connected to vehicle")

# Infinite loop to receive and process position information
while True:
    try:
        # Receive MAVLink message
        msg = master.recv_match().to_dict()

        # Check if the message contains position data
        if msg.get('mavpackettype') == 'GLOBAL_POSITION_INT':
            # Get the current latitude, longitude, and altitude
            current_lat = msg['lat'] / 1e7  # Convert to degrees
            current_lon = msg['lon'] / 1e7  # Convert to degrees
            current_alt = msg['alt'] / 1000  # Convert to meters

            # Calculate X, Y positions in meters using the reference
            x_pos, y_pos = latlon_to_meters(current_lat, current_lon, REFERENCE_LAT, REFERENCE_LON)

            # Z position is simply the altitude as it is
            z_pos = current_alt  # Z is altitude in meters

            # Print the calculated position in meters (X, Y, Z)
            print(f"Position (X, Y, Z): ({x_pos:.2f}, {y_pos:.2f}, {z_pos:.2f})")

        # Check if the message contains attitude data for yaw
        if msg.get('mavpackettype') == 'ATTITUDE':
            yaw_rad = msg['yaw']  # Yaw is in radians
            yaw_deg = degrees(yaw_rad)  # Convert yaw to degrees
            print(f"Yaw (in degrees): {yaw_deg:.2f}")

    except Exception as e:
        print(f"Error: {e}")
    
    # Sleep to avoid flooding with messages
    time.sleep(0.1)
