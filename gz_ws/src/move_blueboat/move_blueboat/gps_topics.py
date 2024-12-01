import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from pymavlink import mavutil
from math import cos

def gps_to_xy(lat, lon, home_lat, home_lon):
    # Simple flat earth projection
    earth_radius = 6371000  # meters
    dLat = (lat - home_lat) * (3.14159 / 180)
    dLon = (lon - home_lon) * (3.14159 / 180)
    x = earth_radius * dLon * cos(home_lat * (3.14159 / 180))
    y = earth_radius * dLat
    return x, y

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_gps = self.create_publisher(String, '/gps_true', 10)
        self.publisher_xy = self.create_publisher(String, '/gps_x_y', 10)
        self.connection = mavutil.mavlink_connection('udpin:localhost:14550')
        self.home_lat = None
        self.home_lon = None
        self.create_timer(1, self.publish_data)  # Timer to regularly call publish_data method

    def publish_data(self):
        msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            if self.home_lat is None and self.home_lon is None:
                self.home_lat, self.home_lon = lat, lon  # Set the starting position
            x, y = gps_to_xy(lat, lon, self.home_lat, self.home_lon)
            gps_data = json.dumps({'latitude': lat, 'longitude': lon})
            xy_data = json.dumps({'x': x, 'y': y})
            self.publisher_gps.publish(String(data=gps_data))
            self.publisher_xy.publish(String(data=xy_data))

def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSNode()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
