import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import json
import math
import numpy as np
import time
from geometry_msgs.msg import (
    TwistStamped,
    Twist,
    Quaternion,
    PoseStamped,
    PointStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, String, UInt16, UInt16MultiArray, Float32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

from std_srvs.srv import SetBool
import tf_transformations

import tf2_ros

from tf2_ros import TransformBroadcaster

import re
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


from .bridge_node import BridgeNode
from .pubs import Pubs
from .subs import Subs

import uuid


class BlueRov(Node):
    def __init__(self):
        super().__init__("blue_rov_node")
        print("BlueRov Node Started")

        self.uuv_name = (
            self.declare_parameter("uuv_name", "bluerov2")
            .get_parameter_value()
            .string_value
        )
        self.nav_frame = (
            self.declare_parameter("nav_frame", "map")
            .get_parameter_value()
            .string_value
        )

       
        self.starttime = time.time()
        self.ROV_name = "bluerov2"
        self.camera = 1500
        self.lights = 1000
        self.pilot_sensitivity = 25
        self.model_base_link = "base_link"
        self.thrusterRanges = [1100.0, 1900.0]  # pwm Todo, test and move to parameter
        self.thrusterInputRanges = [
            -3.0,
            3.0,
        ]  # m/s, r/s Todo, test and move to parameter
        self.time_last_heartbeat = self.get_clock().now()



        # Services
        self.arm_service = self.create_service(
            SetBool, '/bluerov2/arm', self._handle_Arm
        )
        self.disarm_service = self.create_service(
            SetBool, '/bluerov2/disarm', self._handle_Disarm
        )
        self.set_mode_manual_service = self.create_service(
            SetBool, '/bluerov2/setmode/manual', self._setModeManual
        )
        self.set_mode_alt_hold_service = self.create_service(
            SetBool, '/bluerov2/setmode/alt_hold', self._setModeAltHold
        )
        self.set_mode_stabilize_service = self.create_service(
            SetBool, '/bluerov2/setmode/stabilize', self._setModeStabalize
        )
        self.set_mode_position_service = self.create_service(
            SetBool, '/bluerov2/setmode/position', self._setModePosition
        )
        self.set_mode_position_hold_service = self.create_service(
            SetBool, '/bluerov2/setmode/positionHold', self._setModePositionHold
        )

        self.bluerov_armed = False
        self.bluerov_mode = 'no messages'
        self.set_depth = 0.0

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    ############################################
    ########### NODE METHODS ###################
    ############################################

    def _handle_Arm(self, request, response):
        print("arming @--------------------------------------------")
        # self.bridge.arm_throttle(True)
        response.success = True
        response.message = 'Armed'
        return response

    def _handle_Disarm(self, request, response):
        print("disarming @--------------------------------------------")
        # self.bridge.arm_throttle(False)
        response.success = True
        response.message = 'Disarmed'
        return response

    def _setModeManual(self, request, response):
        print("setting mode to manual @--------------------------------------------")
        # self.bridge.set_mode("manual")
        response.success = True
        response.message = 'Mode set to Manual'
        return response

    def _setModeAltHold(self, request, response):
        print("setting mode to AltHold @--------------------------------------------")
        # self.bridge.set_mode("alt_hold")
        response.success = True
        response.message = 'Mode set to AltHold'
        return response

    def _setModeStabalize(self, request, response):
        print("setting mode to Stabilize @--------------------------------------------")
        # self.bridge.set_mode("stabilize")
        response.success = True
        response.message = 'Mode set to Stabilize'
        return response

    def _setModePosition(self, request, response):
        print("setting mode to Position @--------------------------------------------")
        # self.bridge.set_mode("guided")
        response.success = True
        response.message = 'Mode set to Position (Guided)'
        return response

    def _setModePositionHold(self, request, response):
        print(
            "setting mode to PositionHold @--------------------------------------------"
        )
        response.success = True
        response.message = 'Mode set to Position Hold'
        return response


def main(args=None):
    rclpy.init(args=args)
    blue_rov_node = BlueRov()
    rclpy.spin(blue_rov_node)
    blue_rov_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
