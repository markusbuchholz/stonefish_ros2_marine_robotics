import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import NavSatFix
import math
from gekko import GEKKO
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import time

class WayASVController(Node):
    def __init__(self):
        super().__init__('way_asv_controller')

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10, callback_group=self.callback_group)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10, callback_group=self.callback_group)

        self.pwm_motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_pwm_port_joint/cmd_pwm', 10, callback_group=self.callback_group)
        self.pwm_motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_pwm_stbd_joint/cmd_pwm', 10, callback_group=self.callback_group)


        self.mpc_motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/mpc_port_joint/cmd_mpc', 10, callback_group=self.callback_group)
        self.mpc_motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/mpc_stbd_joint/cmd_mpc', 10, callback_group=self.callback_group)
        


        self.navsat_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10, callback_group=self.callback_group)
        self.odometry_subscription = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10, callback_group=self.callback_group)
        self.asv_pos_gps_publisher = self.create_publisher(PointStamped, '/asv_pos_gps', 10, callback_group=self.callback_group)
        self.waypoint_subscription = self.create_subscription(Float64MultiArray, '/waypoints', self.waypoint_callback, 10, callback_group=self.callback_group)


        self.motors_pwm_publisher = self.create_publisher(String, '/blueboat/motors_pwm/cmd_pwm', 10, callback_group=self.callback_group)
        
        #self.reference_position = (-22.986686999936378, -43.2023544348319) #x:15m y:0m
        self.reference_position = (-22.986686723209495, -43.20250343605857) #x:15m y:0m
        

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
        self.angular_kI = 2.0#1.0   
        self.angular_kD = 1.0#0.6  

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
        self.state = 'rotate_to_waypoint'
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
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_yaw)

        self.get_logger().info(f"State: {self.state}, Current Position: {self.current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters, Heading Error: {heading_error:.2f}")

        current_time = time.time()
        delta_time = current_time - self.previous_time

        if self.state == 'rotate_to_waypoint':
            if abs(heading_error) < 0.1:
                self.state = 'move_to_waypoint'
                self.get_logger().info("Transition to state: move_to_waypoint")
            else:
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(0.0, angular_velocity)
        elif self.state == 'move_to_waypoint':
            if distance_to_waypoint < 1.0:
                self.state = 'stop_at_waypoint'
                self.stop_asv()
                self.get_logger().info("Transition to state: stop_at_waypoint")
            else:
                linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_waypoint, self.previous_linear_error, self.linear_integral, delta_time)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
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

    def run_mpc(self):
        x_val, y_val, psi_val, T1_val, T2_val, T1_pwm, T2_pwm = self.solve_mpc(self.current_waypoint_index)
        return T1_val[1], T2_val[1]  # Only return the last thrust values for the current step

    def solve_mpc(self, start_index):
        m = GEKKO(remote=False)
        horizon = min(10, self.total_waypoints - start_index)
        m.time = np.linspace(0, 1, horizon)
        
        # Dynamics and control parameters
        m1, Iz = 13.117, 3.653  # mass and moment of inertia
        Xu, Xuu, Xuuu = -1, -1, -1  # drag coefficients
        Yv, Yr = -1, -1  # sway coefficients
        Nr, Nv = -1, -1  # yaw coefficients
        Kpu = 1  # control gain for surge
        ud = 1  # desired surge velocity
        d = 0.3

        # State variables
        nu = [m.Var(value=0) for _ in range(3)]  # nu = [u, v, r]
        x = m.Var(value=self.current_position[0])  # Starting from current position
        y = m.Var(value=self.current_position[1])  # Starting from current position
        psi = m.Var(value=self.current_yaw)  # Starting from current yaw

        # Control inputs as thrusters now
        T1 = m.MV(value=0, lb=-3, ub=5)  # Adjusted bounds based on the thrust curve
        T2 = m.MV(value=0, lb=-3, ub=5)  # Adjusted bounds based on the thrust curve
        T1.STATUS = 1
        T2.STATUS = 1

        # Reference paths
        ref_x = m.Param(value=self.waypoints_x[start_index:start_index + horizon])
        ref_y = m.Param(value=self.waypoints_y[start_index:start_index + horizon])

        print("Reference X values:", ref_x.value)
        print("Reference Y values:", ref_y.value)

        # Mapping T1 and T2 to emulate Fx and Mz directly
        Fx = T1 + T2
        Mz = (T1 - T2) * d

        ##### Model of waves #####
        rho_water = 1000  # Density of water (kg/m^3)
        g = 9.81  # Acceleration due to gravity (m/s^2)
        L, B, T = 2, 2, 0.5  # Length, Breadth, and Draft of the vehicle
        A, Lambda, omega_e, phi = 3.0, 25000.0, 0.5, 0
        beta = np.pi / 4
        wave_time = m.Param(value=m.time)
        si = m.sin(omega_e * wave_time + phi) * (2 * np.pi / Lambda) * A

        F_wave_x = rho_water * g * B * L * T * m.cos(beta) * si
        F_wave_y = - rho_water * g * B * L * T * m.sin(beta) * si
        F_wave_z =  rho_water * g * B * L * (L**2 - B**2) / 24 * m.sin(2 * beta) * si

        ##### Wind effects #####
        Vw = 1  # Wind speed (m/s)
        beta_w = np.pi / 4  # Wind direction relative to the vehicle's heading
        uw = Vw * m.cos(beta_w - psi)
        vw = Vw * m.sin(beta_w - psi)
        Vrw = m.sqrt(uw**2 + vw**2)
        
        # Workaround for atan2 using atan
        gamma_rw = m.atan(vw / (uw + 1e-8))  # Small value added to avoid division by zero

        rho_air = 1.225  # Density of air (kg/m^3)
        Cx, Cy, Ck = 0.001, 0.001, 0.001
        Aw, Alw, Hlw = 5, 5, 2  # Area and height coefficients
        F_wind_x = 0.5 * rho_air * Vrw**2 * Cx * Aw
        F_wind_y = 0.5 * rho_air * Vrw**2 * Cy * Alw
        M_wind_z = 0.5 * rho_air * Vrw**2 * Ck * Alw * Hlw

        # Current parameters
        Vc = 0.5  # Speed of the current (m/s)
        alpha_c = np.pi / 6  # Angle of the current in inertial frame
        beta_c = np.pi / 4  # Angle of current attack relative to the bow

        vc_x = Vc * m.cos(alpha_c) * m.cos(beta_c)
        vc_y = Vc * m.sin(beta_c)

        # Equations of motion using Fx and Mz
        m.Equations([
            m1 * nu[0].dt() - F_wave_x - F_wind_x == Fx - (Xu + Xuu * abs(nu[0]) + Xuuu * nu[0]**2) + Kpu * (ud - nu[0]),
            m1 * nu[1].dt() - F_wave_y - F_wind_y == 0 - (Yv * nu[1] + Yr * nu[2]),  # Assuming no direct control over Fy by T1 and T2
            Iz * nu[2].dt() - F_wave_z - M_wind_z == Mz - (Nv * nu[1] + Nr * nu[2]),  # Mz instead of Fy for yaw moment
            x.dt() + vc_x == nu[0] * m.cos(psi) - nu[1] * m.sin(psi),
            y.dt() + vc_y == nu[0] * m.sin(psi) + nu[1] * m.cos(psi),
            psi.dt() == nu[2]
        ])

        # Control objective with enhanced cost function considerations
        m.Minimize(10 * ((x - ref_x[-1])**2 + (y - ref_y[-1])**2)   # Path tracking
                    + 0.5 * (T1**2 + T2**2)                         # Control effort
                    + 1 * (F_wave_x**2 + F_wave_y**2)            # Disturbance rejection (waves)
                    + 1 * (F_wind_x**2 + F_wind_y**2)            # Disturbance rejection (wind)
                    + 1 * (vc_x**2 + vc_y**2))                   # Penalty for high current interference

        # Solve
        m.options.IMODE = 6
        m.solve(disp=False)  # Display solver output for debugging

        # Convert thrust values to PWM signals
        T1_pwm = self.thrust_to_pwm(T1.value)
        T2_pwm = self.thrust_to_pwm(T2.value)

        # Debugging output
        #self.get_logger().info(f"Solver status: {m.options.SOLVESTATUS}")
        self.get_logger().info(f"T1 values: {T1.value}")
        self.get_logger().info(f"T2 values: {T2.value}")

        return x.value, y.value, psi.value, T1.value, T2.value, T1_pwm, T2_pwm

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

        T1_mpc, T2_mpc = self.run_mpc()
        self.get_logger().info(f"###CHECK MPC ---- T1 values: {T1_mpc}")
        self.get_logger().info(f"###CHECK MPC ----T2 values: {T2_mpc}")

        # Uncomment these lines if you want to add the MPC thrust values to the calculated thrusts
        # thrust_port += T1_mpc
        # thrust_stbd += T2_mpc

        port_mpc_thrust_msg = Float64()
        stbd_mpc_thrust_msg = Float64()
        port_mpc_thrust_msg.data = T1_mpc
        stbd_mpc_thrust_msg.data = T2_mpc

        self.mpc_motor_port_publisher.publish(port_mpc_thrust_msg)
        self.mpc_motor_stbd_publisher.publish(stbd_mpc_thrust_msg)


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

        self.get_logger().info(f"MPC thrust: Port={T1_mpc}, Starboard={T2_mpc}")
        self.get_logger().info(f"Publishing thrust: Port={thrust_port}, Starboard={thrust_stbd}")




    def stop_asv(self):
        self.publish_twist(0.0, 0.0)

    @staticmethod
    def euler_from_quaternion(quat):
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

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
