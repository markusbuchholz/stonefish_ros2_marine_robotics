import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import NavSatFix
import math
from scipy.interpolate import CubicSpline, interp1d
import numpy as np
import time

class SmoothPathASVController(Node):
    def __init__(self):
        super().__init__('smooth_path_asv_controller')

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10, callback_group=self.callback_group)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10, callback_group=self.callback_group)

        self.pwm_motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_pwm_port_joint/cmd_pwm', 10, callback_group=self.callback_group)
        self.pwm_motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_pwm_stbd_joint/cmd_pwm', 10, callback_group=self.callback_group)

        self.navsat_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10, callback_group=self.callback_group)
        self.odometry_subscription = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10, callback_group=self.callback_group)
        self.asv_pos_gps_publisher = self.create_publisher(PointStamped, '/asv_pos_gps', 10, callback_group=self.callback_group)
        self.waypoint_subscription = self.create_subscription(Float64MultiArray, '/waypoints', self.waypoint_callback, 10, callback_group=self.callback_group)

        self.motors_pwm_publisher = self.create_publisher(String, '/blueboat/motors_pwm/cmd_pwm', 10, callback_group=self.callback_group)

        self.reference_position = (-22.986686723209495, -43.20250343605857)
        self.waypoints = []
        self.current_waypoint = None

        self.total_waypoints = 100
        self.waypoints_x = None
        self.waypoints_y = None

        # PID parameters for linear movement
        self.linear_kP = 0.5
        self.linear_kI = 0.1
        self.linear_kD = 0.05

        # PID parameters for angular movement
        self.angular_kP = 4.0
        self.angular_kI = 4.0
        self.angular_kD = 3.0

        self.current_position = (0, 0)
        self.current_yaw = 0.0
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

    def thrust_to_pwm(self, thrust):
        pwm = self.thrust_to_pwm_interp(thrust)
        pwm_clipped = np.clip(pwm, 1100, 1900)  # Ensure PWM stays within the valid range
        return pwm_clipped

    def interpolate_waypoints(self, waypoints, total_points):
        waypoints = np.array(waypoints)
        if len(waypoints) < 2:
            # If there's only one waypoint, add a very small random value to create a second point
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
        # Accept new waypoints and reset state to start navigating
        self.waypoints = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        self.waypoints_x, self.waypoints_y = self.interpolate_waypoints(self.waypoints, self.total_waypoints)
        self.current_waypoint_index = 0
        self.state = 'move_to_waypoint'
        self.get_logger().info(f"Received new waypoints: {self.waypoints}")
        self.navigate_to_waypoint()

    def navsat_callback(self, msg):
        current_position_gps = (msg.latitude, msg.longitude)
        self.current_position = self.convert_gps_to_xy(current_position_gps)

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = float(self.current_position[0])
        pos_msg.point.y = float(self.current_position[1])

        self.asv_pos_gps_publisher.publish(pos_msg)
        self.navigate_to_waypoint()

    def odometry_callback(self, msg):
        orientation_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.current_yaw = yaw

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
            final_heading_error = self.normalize_angle(1.0 - self.current_yaw)
            if abs(final_heading_error) < 0.2:
                self.stop_asv()
                self.state = 'idle'
                self.get_logger().info("Final heading achieved. Transitioning to idle state.")
            else:
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, final_heading_error, self.previous_angular_error, self.angular_integral, time.time() - self.previous_time)
                self.publish_twist(0.0, angular_velocity)
            return

        if self.current_waypoint_index < len(self.waypoints_x):
            waypoint = (self.waypoints_x[self.current_waypoint_index], self.waypoints_y[self.current_waypoint_index])
        else:
            self.state = 'rotate_to_final_heading'
            self.get_logger().info("Reached final waypoint. Transitioning to rotate_to_final_heading state.")
            return

        distance_to_waypoint = self.calculate_distance(self.current_position, waypoint)
        bearing_to_waypoint = self.calculate_bearing(self.current_position, waypoint)
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_yaw)

        self.get_logger().info(f"State: {self.state}, Current Position: {self.current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters, Heading Error: {heading_error:.2f}")

        current_time = time.time()
        delta_time = current_time - self.previous_time

        if self.state == 'move_to_waypoint':
            if distance_to_waypoint < 1.0 and self.current_waypoint_index < len(self.waypoints_x) - 1:
                self.current_waypoint_index += 1
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}. Moving to next waypoint.")
            else:
                linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_waypoint, self.previous_linear_error, self.linear_integral, delta_time)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(linear_velocity, angular_velocity)

        self.previous_time = current_time

    def calculate_pid(self, kp, ki, kd, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kp * error + ki * integral + kd * derivative
        return output

    def publish_twist(self, linear_velocity, angular_velocity):
        thrust_port = linear_velocity - angular_velocity
        thrust_stbd = linear_velocity + angular_velocity

        max_thrust = 10.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)

        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

        pwm_port_msg = Float64()
        pwm_stbd_msg = Float64()
        pwm_port_msg.data = float(self.thrust_to_pwm(thrust_port))
        pwm_stbd_msg.data = float(self.thrust_to_pwm(thrust_stbd))

        self.pwm_motor_port_publisher.publish(pwm_port_msg)
        self.pwm_motor_stbd_publisher.publish(pwm_stbd_msg)

        # Publish to the combined PWM topic
        motors_pwm_msg = String()
        motors_pwm_msg.data = f"port:{int(pwm_port_msg.data)},stbd:{int(pwm_stbd_msg.data)}"
        self.motors_pwm_publisher.publish(motors_pwm_msg)

    def stop_asv(self):
        self.publish_twist(0.0, 0.0)

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def calculate_bearing(self, pos1, pos2):
        return math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0])

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)

    smooth_path_asv_controller = SmoothPathASVController()

    executor = MultiThreadedExecutor()
    executor.add_node(smooth_path_asv_controller)

    try:
        smooth_path_asv_controller.get_logger().info('ASVController node is running')
        executor.spin()
    except KeyboardInterrupt:
        smooth_path_asv_controller.get_logger().info('ASVController node is shutting down')
    finally:
        smooth_path_asv_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()