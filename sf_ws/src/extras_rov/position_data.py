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
    # Establish MAVLink connection
    connection = mavutil.mavlink_connection('udpin:localhost:14550')
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    home_lat = None
    home_lon = None

    vehicle_data = {}

    # Define the MAVLink message types you want to listen to
    desired_message_types = [
        'GPS_RAW_INT',
        'SCALED_IMU',          # IMU data
        'ATTITUDE',            # INS data
        'LOCAL_POSITION_NED',  # Position data in NED frame
        'ODOMETRY',            # Odometry data (if supported)
        'VFR_HUD',             # Additional position and velocity info
        'SYS_STATUS',          # System status, including sensor health
        # Add other message types as needed
    ]

    print("Listening for MAVLink messages...")

    while True:
        msg = connection.recv_match(blocking=True)
        if msg:
            msg_type = msg.get_type()
            if msg_type in desired_message_types:
                msg_dict = msg.to_dict()
                if msg_type not in vehicle_data:
                    vehicle_data[msg_type] = []
                vehicle_data[msg_type].append(msg_dict)
                print(f"Message received: {msg_type}")
                print(json.dumps(msg_dict, indent=2))  # Pretty print JSON

            # Stop collecting after receiving 100 messages in total
            if sum(len(v) for v in vehicle_data.values()) >= 100:
                break

    # Process GPS Data
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

            print(f"GPS Data - Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, "
                  f"EPH: {eph}, EPV: {epv}, Velocity: {vel}, COG: {cog}, "
                  f"Satellites Visible: {satellites_visible}")

            if home_lat is None and home_lon is None:
                home_lat, home_lon = lat, lon  # Set the starting position
            x, y = convert_gps_to_xy(home_lat, home_lon, lat, lon)
            gps_data_json = json.dumps({'latitude': lat, 'longitude': lon})
            xy_data_json = json.dumps({'x': x, 'y': y})
            print(f"GPS Data JSON: {gps_data_json}")
            print(f"XY Data JSON: {xy_data_json}")

    # Process IMU Data
    if 'SCALED_IMU' in vehicle_data:
        imu_data_list = vehicle_data['SCALED_IMU']
        for imu_data in imu_data_list:
            accel_x = imu_data['xacc'] / 1000  # Convert to m/s^2 if needed
            accel_y = imu_data['yacc'] / 1000
            accel_z = imu_data['zacc'] / 1000
            gyro_x = imu_data['xgyro'] / 1000  # Convert to rad/s if needed
            gyro_y = imu_data['ygyro'] / 1000
            gyro_z = imu_data['zgyro'] / 1000
            mag_x = imu_data['xmag']
            mag_y = imu_data['ymag']
            mag_z = imu_data['zmag']

            print(f"IMU Data - Acceleration: ({accel_x}, {accel_y}, {accel_z}) m/s², "
                  f"Gyro: ({gyro_x}, {gyro_y}, {gyro_z}) rad/s, "
                  f"Magnetometer: ({mag_x}, {mag_y}, {mag_z})")

            imu_data_json = json.dumps({
                'acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
                'gyroscope': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                'magnetometer': {'x': mag_x, 'y': mag_y, 'z': mag_z}
            })
            print(f"IMU Data JSON: {imu_data_json}")

    # Process INS Data (Attitude)
    if 'ATTITUDE' in vehicle_data:
        attitude_data_list = vehicle_data['ATTITUDE']
        for attitude_data in attitude_data_list:
            roll = math.degrees(attitude_data['roll'])    # Convert to degrees
            pitch = math.degrees(attitude_data['pitch'])
            yaw = math.degrees(attitude_data['yaw'])
            rollspeed = attitude_data['rollspeed']
            pitchspeed = attitude_data['pitchspeed']
            yawspeed = attitude_data['yawspeed']

            print(f"Attitude Data - Roll: {roll}°, Pitch: {pitch}°, Yaw: {yaw}°, "
                  f"Roll Speed: {rollspeed} rad/s, Pitch Speed: {pitchspeed} rad/s, "
                  f"Yaw Speed: {yawspeed} rad/s")

            attitude_data_json = json.dumps({
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'roll_speed': rollspeed,
                'pitch_speed': pitchspeed,
                'yaw_speed': yawspeed
            })
            print(f"Attitude Data JSON: {attitude_data_json}")

    # Process Odometry Data
    if 'ODOMETRY' in vehicle_data:
        odom_data_list = vehicle_data['ODOMETRY']
        for odom_data in odom_data_list:
            position_x = odom_data['x']
            position_y = odom_data['y']
            position_z = odom_data['z']
            orientation = odom_data['orientation']  # Quaternion
            linear_velocity_x = odom_data['vx']
            linear_velocity_y = odom_data['vy']
            linear_velocity_z = odom_data['vz']
            angular_velocity_x = odom_data['rollspeed']
            angular_velocity_y = odom_data['pitchspeed']
            angular_velocity_z = odom_data['yawspeed']

            print(f"Odometry Data - Position: ({position_x}, {position_y}, {position_z}) m, "
                  f"Orientation (Quaternion): {orientation}, "
                  f"Linear Velocity: ({linear_velocity_x}, {linear_velocity_y}, {linear_velocity_z}) m/s, "
                  f"Angular Velocity: ({angular_velocity_x}, {angular_velocity_y}, {angular_velocity_z}) rad/s")

            odom_data_json = json.dumps({
                'position': {'x': position_x, 'y': position_y, 'z': position_z},
                'orientation': {
                    'w': orientation[0],
                    'x': orientation[1],
                    'y': orientation[2],
                    'z': orientation[3]
                },
                'linear_velocity': {'x': linear_velocity_x, 'y': linear_velocity_y, 'z': linear_velocity_z},
                'angular_velocity': {'x': angular_velocity_x, 'y': angular_velocity_y, 'z': angular_velocity_z}
            })
            print(f"Odometry Data JSON: {odom_data_json}")

    # Process Local Position (NED)
    if 'LOCAL_POSITION_NED' in vehicle_data:
        local_pos_list = vehicle_data['LOCAL_POSITION_NED']
        for local_pos in local_pos_list:
            north = local_pos['x']
            east = local_pos['y']
            down = local_pos['z']
            vx = local_pos['vx']
            vy = local_pos['vy']
            vz = local_pos['vz']

            print(f"Local Position NED - North: {north} m, East: {east} m, Down: {down} m, "
                  f"Velocity: ({vx}, {vy}, {vz}) m/s")

            local_pos_json = json.dumps({
                'north': north,
                'east': east,
                'down': down,
                'velocity': {'x': vx, 'y': vy, 'z': vz}
            })
            print(f"Local Position NED JSON: {local_pos_json}")

    # Process VFR_HUD Data
    if 'VFR_HUD' in vehicle_data:
        vfr_hud_list = vehicle_data['VFR_HUD']
        for vfr in vfr_hud_list:
            airspeed = vfr['airspeed']
            groundspeed = vfr['groundspeed']
            heading = vfr['heading']
            throttle = vfr['throttle']
            alt = vfr['alt']
            climb = vfr['climb']

            print(f"VFR HUD - Airspeed: {airspeed} m/s, Groundspeed: {groundspeed} m/s, "
                  f"Heading: {heading}°, Throttle: {throttle}%, Altitude: {alt} m, Climb: {climb} m/s")

            vfr_json = json.dumps({
                'airspeed': airspeed,
                'groundspeed': groundspeed,
                'heading': heading,
                'throttle': throttle,
                'altitude': alt,
                'climb': climb
            })
            print(f"VFR HUD JSON: {vfr_json}")

    # Process System Status
    if 'SYS_STATUS' in vehicle_data:
        sys_status_list = vehicle_data['SYS_STATUS']
        for sys_status in sys_status_list:
            onboard_control_sensors_present = sys_status['onboard_control_sensors_present']
            onboard_control_sensors_enabled = sys_status['onboard_control_sensors_enabled']
            onboard_control_sensors_health = sys_status['onboard_control_sensors_health']
            load = sys_status['load'] / 100.0  # Scale as needed

            print(f"System Status - Sensors Present: {onboard_control_sensors_present}, "
                  f"Sensors Enabled: {onboard_control_sensors_enabled}, "
                  f"Sensors Health: {onboard_control_sensors_health}, Load: {load}%")

            sys_status_json = json.dumps({
                'sensors_present': onboard_control_sensors_present,
                'sensors_enabled': onboard_control_sensors_enabled,
                'sensors_health': onboard_control_sensors_health,
                'load': load
            })
            print(f"System Status JSON: {sys_status_json}")

    # You can process additional message types similarly

if __name__ == '__main__':
    main()
