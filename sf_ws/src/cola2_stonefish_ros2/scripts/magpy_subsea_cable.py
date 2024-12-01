#!/usr/bin/env python3

import sys
import xml.etree.ElementTree as ET
import numpy as np
import magpylib as magpy
from magpylib.magnet import Cylinder
from magpylib import Collection
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from stonefish_ros2.msg import INS  # Assuming you have stonefish_ros2 package and INS message

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rov_control_pub = self.create_publisher(Float64MultiArray, '/bluerov/controller/thruster_setpoints_sim', 10)
        self.vertical_joystick_index = 1  # Assuming left analog stick
        self.horizontal_joystick_index = 0  # Assuming left analog stick
        self.forward_backward_joystick_index = 4  # Assuming right analog stick
        self.turn_left_right_joystick_index = 3  # Assuming right analog stick
        self.depth_control_mode = False

    def joy_callback(self, data):
        axes = data.axes
        buttons = data.buttons
        if buttons[4] == 1 and buttons[5] == 1:
            if self.depth_control_mode:
                self.depth_control_mode = False
                self.get_logger().info("DEPTH CONTROL MODE DISABLED!")
            else:
                self.get_logger().info("DEPTH CONTROL MODE ENABLED!")
                rov_control_msg = Float64MultiArray()
                rov_control_msg.data = [0.0, 0.0, 0.0, 0.0, 0.4, 0.4]
                self.depth_control_mode = True
        else:
            vertical = axes[self.vertical_joystick_index]
            horizontal = axes[self.horizontal_joystick_index]
            side = axes[self.turn_left_right_joystick_index]
            forward_backward = axes[self.forward_backward_joystick_index]
            depth = 0.4 if self.depth_control_mode or forward_backward != 0 or side != 0 else vertical * -1
            if side < 0 or horizontal < 0:
                rov_control_msg = Float64MultiArray()
                rov_control_msg.data = [
                    horizontal * -1,
                    side,
                    forward_backward,
                    horizontal * -1 + side + forward_backward,
                    depth,
                    depth
                ]
            else:
                rov_control_msg = Float64MultiArray()
                rov_control_msg.data = [
                    -1 * side,
                    horizontal,
                    horizontal + -1 * side + forward_backward,
                    forward_backward,
                    depth,
                    depth
                ]
            self.rov_control_pub.publish(rov_control_msg)


def ned_to_cartesian(ned_position, origin_lat, origin_lon):
    R = 6371000  # Earth's radius in meters
    lat, lon, alt = ned_position
    origin_lat_rad = np.deg2rad(origin_lat)
    origin_lon_rad = np.deg2rad(origin_lon)
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    x = R * (lon_rad - origin_lon_rad) * np.cos(origin_lat_rad)
    y = R * (lat_rad - origin_lat_rad)
    z = -alt  # Downward is positive in NED
    return [x, y, z]


def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    return [qw, qx, qy, qz]


def parse_scenario(scenario_file, object_name):
    tree = ET.parse(scenario_file)
    root = tree.getroot()
    for static_elem in root.findall(".//static[@name='{}']".format(object_name)):
        origin_elem = static_elem.find("physical/origin")
        position = [float(x) for x in origin_elem.attrib['xyz'].split()]
        orientation = [float(x) for x in origin_elem.attrib['rpy'].split()]
        world_transform_elem = static_elem.find("world_transform")
        world_position = [float(x) for x in world_transform_elem.attrib['xyz'].split()]
        world_orientation = [float(x) for x in world_transform_elem.attrib['rpy'].split()]
        position = [pos + world_pos for pos, world_pos in zip(position, world_position)]
        orientation = [ori + world_ori for ori, world_ori in zip(orientation, world_orientation)]
        return position, orientation
    return None, None


class MagpySubseaCable(Node):
    def __init__(self):
        super().__init__('magpy_subsea_cable')
        self.robot_position = None
        self.magnetic_objects = None

        args = self.declare_parameters(
            namespace='',
            parameters=[
                ('scenario_description', ''),
                ('subsea_cable_length', 0.0),
                ('subsea_cable_diameter', 0.0),
                ('magnetic_object_names', [])
            ]
        )

        scenario_description = self.get_parameter('scenario_description').get_parameter_value().string_value
        subsea_cable_length = self.get_parameter('subsea_cable_length').get_parameter_value().double_value
        subsea_cable_diameter = self.get_parameter('subsea_cable_diameter').get_parameter_value().double_value
        magnetic_object_names = self.get_parameter('magnetic_object_names').get_parameter_value().string_array_value

        self.get_logger().info(scenario_description)

        for obj_name in magnetic_object_names:
            position, orientation = parse_scenario(scenario_description, obj_name)
            self.get_logger().info(obj_name)
            if position is not None and orientation is not None:
                quat_orientation = euler_to_quaternion(*orientation)
                orientation_rot = R.from_quat(quat_orientation)
                if obj_name == 'Boatlanding':
                    cyl1 = Cylinder(dimension=(subsea_cable_diameter, subsea_cable_length),
                                    position=(position[0], position[1], position[2]),
                                    orientation=orientation_rot.as_rotvec(),
                                    magnetization=(0.1, 0, 0))
                    self.magnetic_objects = Collection(cyl1)

        self.create_subscription(INS, '/bluerov/navigator/ins', self.robot_position_callback, 10)
        self.magnetic_field_pub = self.create_publisher(Vector3, 'magnetic_field', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def robot_position_callback(self, msg):
        self.robot_position = ned_to_cartesian([msg.latitude, msg.longitude, msg.altitude], msg.origin_latitude, msg.origin_longitude)

    def timer_callback(self):
        if self.robot_position is not None and self.magnetic_objects is not None:
            field = self.magnetic_objects.getB(self.robot_position)
            self.magnetic_field_pub.publish(Vector3(x=field[0], y=field[1], z=field[2]))


def main(args=None):
    rclpy.init(args=args)
    node = MagpySubseaCable()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

