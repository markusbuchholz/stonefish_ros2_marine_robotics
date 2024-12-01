#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import math
import yaml
from scipy.interpolate import CubicSpline
import cvxpy as cp



####################

N = 500  # Number of time steps in the prediction horizon
dt = 0.1  # Time step duration


# Vehicle parameters
m = 500  # Mass of the ASV
Iz = 1000  # Moment of inertia around the z-axis
d1, d2, d3 = 50, 70, 80  # Damping coefficients

# Generate a figure-eight reference trajectory
time = np.linspace(0, N * dt, N)
#p_x_ref = 200 * np.sin(2 * np.pi * 0.02 * time)
#p_y_ref = 200 * np.sin(2 * np.pi * 0.04 * time) * np.cos(2 * np.pi * 0.02 * time)
p_x_ref = time
p_y_ref = 200 * np.sin(2 * np.pi * 0.02 * time)

# radius = 100
# theta = np.linspace(0, 2 * np.pi, N)
# p_x_ref = radius * np.cos(theta)
# p_y_ref = radius * np.sin(theta)


# Initialize state and control variables
#x = cp.Variable((6, N + 1))
#u = cp.Variable((3, N))

# State transition matrix for discrete time
A = np.eye(6)
A[0, 2] = dt
A[1, 3] = dt
A[2, 5] = dt

# Control input matrix for discrete time
B = np.zeros((6, 3))
B[2, 0] = dt / m
B[3, 1] = dt / m
B[5, 2] = dt / Iz

###################

def quaternion_to_yaw(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)
        self.odometry_subscriber = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10)

        self.N = 500  # MPC prediction horizon
        self.dt = 0.1  # MPC time step duration
        self.m = 500   # ASV mass
        self.Iz = 1000 # ASV moment of inertia around the z-axis
        self.d1, self.d2, self.d3 = 50, 70, 80  # ASV damping coefficients
        self.Q = np.diag([10000, 10000, 100, 100, 100, 100])  # State weighting
        self.R = np.diag([0.1, 0.1, 0.1])  # Control input weighting

        # Initialize PID controller variables
        self.Kp = 0.25
        self.Ki = 0.0
        self.Kd = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.constant_forward_speed = 0.25 / 1


        # Load waypoints and generate spline path
        waypoints = self.load_waypoints_from_yaml('waypoints_asv.yaml')
        self.smooth_x, self.smooth_y = self.generate_spline_path(waypoints)

    def load_waypoints_from_yaml(self, file_path):
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            waypoints_data = yaml_data['waypoints']
            waypoints = [ [wp['point'][0], wp['point'][1]] for wp in waypoints_data ]
            return np.array(waypoints)

    def generate_spline_path(self, waypoints):
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        distance = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0)
        spline_x = CubicSpline(distance, x)
        spline_y = CubicSpline(distance, y)
        fine_distance = np.linspace(0, distance[-1], 1000)
        return spline_x(fine_distance), spline_y(fine_distance)

    def odometry_callback(self, data):
        current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        current_yaw = quaternion_to_yaw(data.pose.pose.orientation)
        self.control_robot(current_position, current_yaw)
        self.log_position_and_yaw(current_position, current_yaw)

    def calculate_thrust_commands(self, surge_force, yaw_moment, pid_adjustment):
        # This method should contain the logic to convert surge force, yaw moment, 
        # and PID adjustment into left and right thrust commands.
        # The following is a placeholder example:

        # Example conversion logic (adjust according to your system's specifics)
        left_thrust = surge_force + yaw_moment - pid_adjustment
        right_thrust = surge_force - yaw_moment + pid_adjustment
        

        # Ensure the thrust values are within allowable limits
        #left_thrust = max(min(left_thrust, max_thrust), min_thrust)
        #right_thrust = max(min(right_thrust, max_thrust), min_thrust)

        return left_thrust, right_thrust

    def control_robot(self, current_position, current_yaw):
        distances = np.sqrt((self.smooth_x - current_position[0])**2 + (self.smooth_y - current_position[1])**2)
        closest_index = np.argmin(distances)

        # Use MPC for optimal trajectory following
        surge_force, sway_force, yaw_moment = self.calculate_mpc(current_position, current_yaw, closest_index)

        # LOS and PID controller for real-time adjustments
        lookahead_distance = 1.0 * 1
        cumulative_distances = np.cumsum(np.sqrt(np.diff(self.smooth_x)**2 + np.diff(self.smooth_y)**2))
        lookahead_index = np.searchsorted(cumulative_distances, cumulative_distances[closest_index] + lookahead_distance)
        lookahead_index = min(lookahead_index, len(self.smooth_x) - 1)

        lookahead_point = (self.smooth_x[lookahead_index], self.smooth_y[lookahead_index])
        los_angle = math.atan2(lookahead_point[1] - current_position[1], lookahead_point[0] - current_position[0])
        heading_error = math.atan2(math.sin(los_angle - current_yaw), math.cos(los_angle - current_yaw))

        self.integral_error += heading_error
        derivative_error = heading_error - self.prev_error

        pid_adjustment = self.Kp * heading_error + self.Ki * self.integral_error + self.Kd * derivative_error
        self.prev_error = heading_error

        # Combine MPC and PID outputs for actuation
        thrust_scale_factor = 0.1

        left_thrust, right_thrust = self.calculate_thrust_commands(surge_force, yaw_moment, pid_adjustment)
        self.publish_thrust(left_thrust * thrust_scale_factor, right_thrust* thrust_scale_factor)
        self.log_thrust_values(left_thrust* thrust_scale_factor, right_thrust* thrust_scale_factor)



    def calculate_mpc(self, current_position, current_yaw, closest_index):
        # Define the MPC problem
        horizon_length = 200

        x = cp.Variable((6, horizon_length + 1))
        u = cp.Variable((3, horizon_length))

        # State transition matrix for discrete time
        A = np.eye(6)
        A[0, 2] = self.dt
        A[1, 3] = self.dt
        A[2, 5] = self.dt

        # Control input matrix for discrete time
        B = np.zeros((6, 3))
        B[2, 0] = self.dt / self.m
        B[3, 1] = self.dt / self.m
        B[5, 2] = self.dt / self.Iz



        # Determine the closest index to the current position in the global trajectory
        distances = np.sqrt((self.smooth_x - current_position[0])**2 + (self.smooth_y - current_position[1])**2)
        closest_index = np.argmin(distances)


        # Cost function and constraints
        cost = 0
        constraints = [x[:, 0] == np.array([current_position[0], current_position[1], current_yaw, 0, 0, 0])]

        for k in range(horizon_length):
            # Determine the index of the reference state in the global trajectory
            global_index = closest_index + k
            if global_index >= len(self.smooth_x):
                global_index = len(self.smooth_x) - 1  # Use the last point if the end is reached

            # Calculate reference state at step k
            ref_state = np.array([self.smooth_x[global_index], 
                                self.smooth_y[global_index], 
                                0, 0, 0, 0])  # Assuming a constant reference yaw and velocities

            # State deviation from the reference
            cost += cp.quad_form(x[:2, k] - ref_state[:2], self.Q[:2, :2])
            # Penalize other states and control inputs
            cost += cp.quad_form(x[2:, k], self.Q[2:, 2:])
            cost += cp.quad_form(u[:, k], self.R)

            # Dynamics constraints
            constraints += [x[:, k + 1] == A @ x[:, k] + B @ u[:, k]]

        # Solve the MPC problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, verbose=False)

        if prob.status in ["optimal", "optimal_inaccurate"]:
            # Extract the first set of control actions
            optimal_controls = u[:, 0].value
            return optimal_controls[0], optimal_controls[1], optimal_controls[2]
        else:
            self.get_logger().error("MPC solution not found.")
            return 0, 0, 0


    def translate_to_thrust(self, surge_force, sway_force, yaw_moment):
        # [Translate surge force, sway force, and yaw moment to left and right thrust commands...]
        # Placeholder logic
        left_thrust = surge_force / self.m  # Example calculation
        right_thrust = yaw_moment / self.Iz  # Example calculation
        return left_thrust, right_thrust

    def publish_thrust(self, left_thrust, right_thrust):
        port_thrust = Float64()
        stbd_thrust = Float64()
        port_thrust.data = left_thrust
        stbd_thrust.data = right_thrust
        self.motor_port_publisher.publish(port_thrust)
        self.motor_stbd_publisher.publish(stbd_thrust)

    def log_position_and_yaw(self, position, yaw):
        self.get_logger().info(f"Current position: x={position[0]}, y={position[1]}, Current yaw: {math.degrees(yaw):.2f} degrees")

    def log_thrust_values(self, left_thrust, right_thrust):
        self.get_logger().info(f"Left Thrust: {left_thrust}, Right Thrust: {right_thrust}")

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
