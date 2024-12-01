import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
import math

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class ASVController(Node):
    def __init__(self):
        super().__init__('asv_controller')

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)

        self.navsat_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.asv_pos_gps_publisher = self.create_publisher(PointStamped, '/asv_pos_gps', 10)

        self.reference_position = (-22.986686999936378, -43.2023544348319)  # Reference GPS position

        # Define waypoints in XY coordinates (meters) relative to the origin (0, 0)
        self.waypoints = [(10, 0), (20, 0), (10, 17.32), (0, 0)]  # Equilateral triangle with 20m sides
        self.current_waypoint_index = 0

        # Tuning PID controllers
        self.linear_pid = PIDController(kp=0.5, ki=0.0, kd=0.05)  # Tuning for smoother control
        self.angular_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)  # Tuning for smoother control

        self.last_time = self.get_clock().now()

    def navsat_callback(self, msg):
        current_position_gps = (msg.latitude, msg.longitude)
        current_position = self.convert_gps_to_xy(current_position_gps)

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = current_position[0]
        pos_msg.point.y = current_position[1]

        self.asv_pos_gps_publisher.publish(pos_msg)

        self.navigate_to_waypoint(current_position)

    def convert_gps_to_xy(self, gps_position):
        ref_lat, ref_lon = self.reference_position
        lat, lon = gps_position

        delta_lat = lat - ref_lat
        delta_lon = lon - ref_lon

        # Approximate conversions from degrees to meters
        meters_per_degree_lat = 111320
        meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(ref_lat))

        x = delta_lon * meters_per_degree_lon
        y = delta_lat * meters_per_degree_lat

        return (x, y)

    def navigate_to_waypoint(self, current_position):
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_asv()
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        distance_to_waypoint = self.calculate_distance(current_position, waypoint)
        bearing_to_waypoint = self.calculate_bearing(current_position, waypoint)

        if distance_to_waypoint < 1.0:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.stop_asv()
                return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # LOS guidance
        heading = bearing_to_waypoint  # Simplified LOS heading

        # Calculate control signals using PID
        linear_velocity = self.linear_pid.compute(distance_to_waypoint, dt)
        angular_velocity = self.angular_pid.compute(heading, dt)

        max_linear_velocity = 2.0
        max_angular_velocity = 1.0
        linear_velocity = max(min(linear_velocity, max_linear_velocity), -max_linear_velocity)
        angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

        self.cmd_vel_callback(twist_msg)

        self.last_time = current_time

        print(f"Current Position: {current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        k_linear = 1.0
        k_angular = 1.0

        thrust_port = k_linear * linear_x - k_angular * angular_z
        thrust_stbd = k_linear * linear_x + k_angular * angular_z

        max_thrust = 10.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)

        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = -thrust_port  # Adjusted as per your suggestion
        stbd_thrust_msg.data = -thrust_stbd  # Adjusted as per your suggestion

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

    def stop_asv(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_callback(stop_msg)

    @staticmethod
    def calculate_distance(pointA, pointB):
        return math.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)

    @staticmethod
    def calculate_bearing(pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB

        angle = math.atan2(y2 - y1, x2 - x1)
        bearing = math.degrees(angle)
        bearing = (bearing + 360) % 360  # Normalize to 0-360 degrees

        return bearing

def main(args=None):
    rclpy.init(args=args)
    node = ASVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
