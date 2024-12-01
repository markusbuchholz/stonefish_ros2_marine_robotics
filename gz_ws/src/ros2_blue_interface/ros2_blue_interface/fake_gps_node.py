import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil
import time
from pyproj import Transformer

class GPSInjector(Node):
    def __init__(self):
        super().__init__('gps_injector')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/aruco_positions_xy',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # MAVLink setup
        self.master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
        self.master.wait_heartbeat()  # Wait for a heartbeat to confirm the connection
        
        # Transformer setup
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32610")
        self.base_latitude = 34.5678901
        self.base_longitude = -120.1234567
        self.base_easting, self.base_northing = self.transformer.transform(self.base_latitude, self.base_longitude)

    def listener_callback(self, msg):
        # Parse X and Y from the Float32MultiArray
        if len(msg.data) >= 3:  # Check if the message contains at least three elements
            x = msg.data[1]  # X coordinate
            y = msg.data[2]  # Y coordinate
            lat, lon = self.xy_to_gps(x, y)
            self.get_logger().info(f'Received XY ({x}, {y}); Converted to lat {lat} lon {lon}')
            self.send_gps_input(lat, lon)
        else:
            self.get_logger().error('Received data does not contain enough elements.')

    def xy_to_gps(self, x, y):
        new_easting = self.base_easting + x
        new_northing = self.base_northing + y
        latitude, longitude = self.transformer.transform(new_northing, new_easting, direction='INVERSE')
        return int(latitude * 1E7), int(longitude * 1E7)

    def send_gps_input(self, lat, lon, alt=10000, hdop=1.0, vdop=1.0):
        gps_week_seconds = int(time.time() - 315964800) % 604800
        gps_week = int((time.time() - 315964800) / 604800)
        self.master.mav.gps_input_send(
            int(time.time() * 1E6), 0,
            (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
             mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
             mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
            gps_week_seconds * 1000, gps_week, 3,
            lat, lon, alt, hdop, vdop, 0, 0, 0, 0, 0, 0, 10
        )

def main(args=None):
    rclpy.init(args=args)
    gps_injector = GPSInjector()
    rclpy.spin(gps_injector)
    gps_injector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
