from pymavlink import mavutil
import time
from pyproj import Transformer

# Initialize the transformer from pyproj
transformer = Transformer.from_crs("epsg:4326", "epsg:32610")  # From WGS84 to UTM Zone 10N

# Base GPS coordinates as the central point of your operations
base_latitude = 34.5678901
base_longitude = -120.1234567

# Convert base latitude and longitude to UTM
base_easting, base_northing = transformer.transform(base_latitude, base_longitude)

# MAVLink connection
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat()  # Wait for a heartbeat to confirm the connection

def xy_to_gps(x, y):
    """
    Converts local XY coordinates (in meters) to GPS coordinates.
    """
    # Calculate new UTM coordinates based on the base point
    new_easting = base_easting + x
    new_northing = base_northing + y

    # Convert back to latitude and longitude
    latitude, longitude = transformer.transform(new_northing, new_easting, direction='INVERSE')
    return int(latitude * 1E7), int(longitude * 1E7)

def send_gps_input(lat, lon, alt=10000, hdop=1.0, vdop=1.0):
    # Ensure latitude and longitude are within the valid range
    if not (-2147483648 <= lat <= 2147483647) or not (-2147483648 <= lon <= 2147483647):
        raise ValueError("Latitude or Longitude out of range.")

    gps_week_seconds = int(time.time() - 315964800) % 604800  # GPS time since Jan 6, 1980
    gps_week = int((time.time() - 315964800) / 604800)

    # Convert latitude and longitude to integers explicitly
    lat = int(lat)
    lon = int(lon)

    # Sending the GPS input data
    master.mav.gps_input_send(
        int(time.time() * 1E6),  # Current timestamp in microseconds
        0,  # GPS ID, assuming a single GPS input
        (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),  # Ignore velocity and speed accuracy
        gps_week_seconds * 1000,  # GPS time in ms from the start of the GPS week
        gps_week,  # GPS week number
        3,  # GPS fix type, assuming a 3D fix
        lat,  # Latitude in degrees * 1E7
        lon,  # Longitude in degrees * 1E7
        alt,  # Altitude in meters (AMSL)
        hdop,  # HDOP in meters
        vdop,  # VDOP in meters
        0,  # North velocity (ignored)
        0,  # East velocity (ignored)
        0,  # Down velocity (ignored)
        0,  # Speed accuracy (ignored)
        0,  # Horizontal accuracy in meters
        0,  # Vertical accuracy in meters
        10  # Number of satellites visible (example value, adjust as needed)
    )



def get_xy_from_camera():
    return 2, 2

while True:
    x, y = get_xy_from_camera()
    lat, lon = xy_to_gps(x, y)
    print("lat ", lat, "lon :", lon)
    send_gps_input(lat, lon)
    time.sleep(1)  
