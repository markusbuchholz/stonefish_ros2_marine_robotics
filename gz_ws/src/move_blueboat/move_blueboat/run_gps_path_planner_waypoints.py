import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, Float64MultiArray, String, Float32
import math
import json
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class WayASVController(Node):
    def __init__(self):
        super().__init__('way_asv_controller')
        
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray, 
            '/waypoints', 
            self.waypoint_callback, 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )

        self.gps_subscription = self.create_subscription(
            String, 
            '/gps_true', 
            self.gps_callback, 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )

        self.port_thrust_publisher = self.create_publisher(
            Float32, 
            '/blueboat/send_port_motor_0_100_thrust', 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )

        self.stbd_thrust_publisher = self.create_publisher(
            Float32, 
            '/blueboat/send_stbd_motor_0_100_thrust', 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )

        self.reference_position = (-22.986686723209495, -43.20250343605857) #x:15m y:0m
        
        self.waypoints = []
        self.current_waypoint = None

        self.total_waypoints = 100
        self.waypoints_x = None
        self.waypoints_y = None

        self.linear_kP = 0.5
        self.linear_kI = 0.1
        self.linear_kD = 0.05

        self.angular_kP = 4.0  
        self.angular_kI = 2.0   
        self.angular_kD = 1.0  

        self.current_position = (0, 0)
        self.previous_position = (0, 0)  # Store the previous GPS position
        self.current_yaw_navsat = 0.0
        self.yaw_offset = None  # Yaw offset between NavSatFix and Odometry
        self.state = 'idle'

        self.current_waypoint_index = 0

        self.target_heading = None  # Target heading for the final rotation

        self.thrust_kgf = np.array([-2.79, -2.21, -1.42, -0.82, -0.24, 0.0, 0.5, 1.17, 1.93, 2.37, 2.76, 3.57, 4.36, 5.22, 5.63])
        self.pwm_us = np.array([1110, 1188, 1292, 1370, 1448, 1500, 1552, 1604, 1656, 1682, 1708, 1760, 1812, 1864, 1900])
        self.thrust_to_pwm_interp = interp1d(self.thrust_kgf, self.pwm_us, kind='linear', fill_value="extrapolate")

        self.previous_time = time.time()
        self.linear_integral = 0.0
        self.previous_linear_error = 0.0
        self.angular_integral = 0.0
        self.previous_angular_error = 0.0

        # Timer for constant publishing
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def thrust_to_pwm(self, thrust):
        pwm = self.thrust_to_pwm_interp(thrust)
        pwm_clipped = np.clip(pwm, 1100, 1900)  # Ensure PWM stays within the valid range
        return pwm_clipped

    def interpolate_waypoints(self, waypoints, total_points):
        waypoints = np.array(waypoints)
        if len(waypoints) < 2:
            tiny_offset = np.random.normal(scale=1e-6, size=2)
            second_point = waypoints[0] + tiny_offset
            waypoints = np.vstack([waypoints, second_point])
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        t = np.linspace(0, 1, len(x))
        t_new = np.linspace(0, 1, total_points)
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
        return x_new, y_new

    def waypoint_callback(self, msg):
        self.waypoints = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        self.waypoints_x, self.waypoints_y = self.interpolate_waypoints(self.waypoints, self.total_waypoints)
        self.current_waypoint_index = 0
        self.state = 'rotate_to_waypoint'
        self.get_logger().info(f"Received new waypoints: {self.waypoints}")
        self.navigate_to_waypoint()

    def gps_callback(self, msg):
        data = json.loads(msg.data)
        current_position_gps = (data['latitude'], data['longitude'])
        self.current_position = self.convert_gps_to_xy(current_position_gps)
        
        if self.previous_position != (0, 0):  # Skip if it's the first GPS reading
            # Calculate yaw from previous GPS position to current GPS position
            raw_yaw_navsat = self.calculate_bearing(self.previous_position, self.current_position)
            if self.yaw_offset is None:
                self.yaw_offset = self.current_yaw_navsat - raw_yaw_navsat
            self.current_yaw_navsat = self.normalize_angle(raw_yaw_navsat + self.yaw_offset)

        self.previous_position = self.current_position  # Update the previous position

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = float(self.current_position[0])
        pos_msg.point.y = float(self.current_position[1])

        self.asv_pos_gps_publisher.publish(pos_msg)
        self.navigate_to_waypoint()

    def convert_gps_to_xy(self, gps_position):
        ref_lat, ref_lon = self.reference_position
        lat, lon = gps_position

        delta_lat = lat - ref_lat
        delta_lon = lon - ref_lon

        meters_per_degree_lat = 111320
        meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(ref_lat))

        x = delta_lon * meters_per_degree_lon
        y = delta_lat * meters_per_degree_lat

        return x, y
    
    def navigate_to_waypoint(self):
        if self.state == 'idle' or self.waypoints_x is None or self.waypoints_y is None:
            return

        if self.state == 'rotate_to_final_heading':
            final_heading_error = self.normalize_angle(1.0 - self.current_yaw_navsat)
            if abs(final_heading_error) < 0.1:
                self.stop_asv()
                self.state = 'idle'
                self.get_logger().info("Final heading achieved. Transitioning to idle state.")
            else:
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, final_heading_error, self.previous_angular_error, self.angular_integral, time.time() - self.previous_time)
                self.publish_twist(0.0, angular_velocity)
            return

        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
        else:
            return

        distance_to_waypoint = self.calculate_distance(self.current_position, waypoint)
        bearing_to_waypoint = self.calculate_bearing(self.current_position, waypoint)
        heading_error_navsat = self.normalize_angle(bearing_to_waypoint - self.current_yaw_navsat)

        self.get_logger().info(f"State: {self.state}, Current Position: {self.current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters")
        self.get_logger().info(f"Heading Error (NavSat): {heading_error_navsat:.2f}")

        current_time = time.time()
        delta_time = current_time - self.previous_time

        if self.state == 'rotate_to_waypoint':
            if abs(heading_error_navsat) < 0.1:
                self.state = 'move_to_waypoint'
                self.get_logger().info("Transition to state: move_to_waypoint")
            else:
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error_navsat, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(0.0, angular_velocity)
        elif self.state == 'move_to_waypoint':
            if distance_to_waypoint < 1.0:
                self.state = 'stop_at_waypoint'
                self.stop_asv()
                self.get_logger().info("Transition to state: stop_at_waypoint")
            else:
                linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_waypoint, self.previous_linear_error, self.linear_integral, delta_time)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error_navsat, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(linear_velocity, angular_velocity)
        elif self.state == 'stop_at_waypoint':
            self.stop_asv()
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                self.state = 'rotate_to_waypoint'
                self.get_logger().info("Transition to state: rotate_to_waypoint")
            else:
                self.rotate_to_heading(1.0)  # Set the desired final heading here, e.g., 1.0 radians
                self.state = 'rotate_to_final_heading'
                self.get_logger().info("Transition to state: rotate_to_final_heading")

        self.previous_time = current_time

    def rotate_to_heading(self, target_heading):
        self.target_heading = target_heading
        self.state = 'rotate_to_heading'

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output

    def publish_twist(self, linear_x, angular_z):
        thrust_port = linear_x - angular_z
        thrust_stbd = linear_x + angular_z

        max_thrust = 10.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)

        port_float = float(self.thrust_to_pwm(thrust_port))
        stbd_float = float(self.thrust_to_pwm(thrust_stbd))

        port_0_100 = (port_float - 1100) / 8.0
        stbd_0_100 = (stbd_float - 1100) / 8.0

        self.get_logger().info(f"Publishing thrust: Port={port_0_100}, Starboard={stbd_0_100}")

        port_0_100_msg = Float32()
        stbd_0_100_msg = Float32()
        port_0_100_msg.data = float(port_0_100)
        stbd_0_100_msg.data = float(stbd_0_100)

        self.port_thrust_publisher.publish(port_0_100_msg)
        self.stbd_thrust_publisher.publish(stbd_0_100_msg)

    def timer_callback(self):
        # Publish messages periodically
        self.navigate_to_waypoint()

    def stop_asv(self):
        self.publish_twist(0.0, 0.0)

    @staticmethod
    def calculate_distance(pointA, pointB):
        return math.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)

    @staticmethod
    def calculate_bearing(pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        angle = math.atan2(y2 - y1, x2 - x1)
        return angle

    @staticmethod
    def normalize_angle(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    way_asv_controller = WayASVController()
    executor = MultiThreadedExecutor()
    executor.add_node(way_asv_controller)

    try:
        way_asv_controller.get_logger().info('ASVController node is running')
        executor.spin()
    except KeyboardInterrupt:
        way_asv_controller.get_logger().info('ASVController node is shutting down')
    finally:
        way_asv_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
