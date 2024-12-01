#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from scipy.interpolate import CubicSpline
import yaml

class RobotController:
    def __init__(self):
        self.Kp = 0.05 #0.025 #0.05  # Proportional gain for heading control
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.0  # Derivative gain
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.constant_forward_speed = 0.25 / 11 # Reduced speed by a factor of 4
        self.waypoint_tolerance = 0.25

    def load_waypoints_from_yaml(self, file_path):
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            waypoints_data = yaml_data['waypoints']
            waypoints = []
            for wp in waypoints_data:
                point = wp['point']
                waypoints.append([point[0], point[1]])  # X, Y coordinates
            return np.array(waypoints)

    def generate_spline_path(self, waypoints):
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        distance = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0)
        spline_x = CubicSpline(distance, x)
        spline_y = CubicSpline(distance, y)
        fine_distance = np.linspace(0, distance[-1], 1000)
        smooth_x = spline_x(fine_distance)
        smooth_y = spline_y(fine_distance)
        return smooth_x, smooth_y

    def quaternion_to_yaw(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def control_robot(self, pub_left, pub_right, current_position, current_yaw, smooth_x, smooth_y):
        distances = np.sqrt((smooth_x - current_position[0])**2 + (smooth_y - current_position[1])**2)
        closest_index = np.argmin(distances)

        if closest_index >= len(smooth_x) - 1:
            pub_left.publish(Float32(0))
            pub_right.publish(Float32(0))
            rospy.loginfo("Last waypoint reached. Stopping the robot.")
            return

        lookahead_distance = 1.0
        cumulative_distances = np.cumsum(np.sqrt(np.diff(smooth_x)**2 + np.diff(smooth_y)**2))
        lookahead_index = np.searchsorted(cumulative_distances, cumulative_distances[closest_index] + lookahead_distance)
        lookahead_index = min(lookahead_index, len(smooth_x) - 1)

        lookahead_point = (smooth_x[lookahead_index], smooth_y[lookahead_index])

        los_angle = math.atan2(lookahead_point[1] - current_position[1], lookahead_point[0] - current_position[0])
        heading_error = math.atan2(math.sin(los_angle - current_yaw), math.cos(los_angle - current_yaw))

        # PID controller
        self.integral_error += heading_error
        derivative_error = heading_error - self.prev_error

        thrust_adjustment = self.Kp * heading_error + self.Ki * self.integral_error + self.Kd * derivative_error
        self.prev_error = heading_error

        left_thrust = max(min(self.constant_forward_speed - thrust_adjustment, 1.0), 0.0)
        right_thrust = max(min(self.constant_forward_speed + thrust_adjustment, 1.0), 0.0)

        pub_left.publish(Float32(left_thrust))
        pub_right.publish(Float32(right_thrust))

        rospy.loginfo("Current position: {}, Current yaw: {:.2f}".format(current_position, math.degrees(current_yaw)))
        rospy.loginfo("Heading towards waypoint: {}".format(lookahead_point))

    def move_robot(self, smooth_x, smooth_y):
        rospy.init_node('robot_path_follower', anonymous=True)
        pub_left = rospy.Publisher('/left_thrust_cmd', Float32, queue_size=10)
        pub_right = rospy.Publisher('/right_thrust_cmd', Float32, queue_size=10)

        def pose_callback(data):
            current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
            current_yaw = self.quaternion_to_yaw(data.pose.pose.orientation)
            self.control_robot(pub_left, pub_right, current_position, current_yaw, smooth_x, smooth_y)

        rospy.Subscriber('/p3d', Odometry, pose_callback)
        rospy.spin()

if __name__ == '__main__':
    controller = RobotController()
    waypoints = controller.load_waypoints_from_yaml('waypoints_asv.yaml')
    smooth_x, smooth_y = controller.generate_spline_path(waypoints)
    try:
        controller.move_robot(smooth_x, smooth_y)
    except rospy.ROSInterruptException:
        pass