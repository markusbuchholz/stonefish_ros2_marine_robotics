import json
from pymavlink import mavutil
import math

def convert_gps_to_xy(ref_lat, ref_lon, lat, lon):
    delta_lat = lat - ref_lat
    delta_lon = lon - ref_lon

    meters_per_degree_lat = 111320
    meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(ref_lat))

    x = delta_lon * meters_per_degree_lon
    y = delta_lat * meters_per_degree_lat

    return x, y

def main():
    connection = mavutil.mavlink_connection('udpin:localhost:14550')
    home_lat = None
    home_lon = None

    vehicle_data = {}

    while True:
        msg = connection.recv_match(blocking=True)
        if msg:
            msg_dict = msg.to_dict()
            msg_type = msg.get_type()
            if msg_type not in vehicle_data:
                vehicle_data[msg_type] = []
            vehicle_data[msg_type].append(msg_dict)
            print(f"Message received: {msg_type}")
            print(msg_dict)

        # Assuming you want to stop collecting after a certain condition
        # For example, after receiving 100 messages
        if sum(len(v) for v in vehicle_data.values()) >= 100:
            break

    # Extract and print GPS data if available
    if 'GPS_RAW_INT' in vehicle_data:
        gps_data_list = vehicle_data['GPS_RAW_INT']
        for gps_data in gps_data_list:
            lat = gps_data['lat'] / 1e7
            lon = gps_data['lon'] / 1e7
            alt = gps_data['alt'] / 1000  # Convert millimeters to meters
            eph = gps_data['eph'] / 100
            epv = gps_data['epv'] / 100
            vel = gps_data['vel'] / 100
            cog = gps_data['cog'] / 100
            satellites_visible = gps_data['satellites_visible']

            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, EPH: {eph}, EPV: {epv}, Velocity: {vel}, COG: {cog}, Satellites Visible: {satellites_visible}")

            if home_lat is None and home_lon is None:
                home_lat, home_lon = lat, lon  # Set the starting position
            x, y = convert_gps_to_xy(home_lat, home_lon, lat, lon)
            gps_data_json = json.dumps({'latitude': lat, 'longitude': lon})
            xy_data_json = json.dumps({'x': x, 'y': y})
            print(f"GPS Data: {gps_data_json}")
            print(f"XY Data: {xy_data_json}")

if __name__ == '__main__':
    main()
