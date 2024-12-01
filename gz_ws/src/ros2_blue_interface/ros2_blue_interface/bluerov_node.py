## Jonatan Scharff Willners @2023
## Markus Buchholz @2024 @ROS 2

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
    Vector3
)
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, String, UInt16, UInt16MultiArray, Float32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import SetBool
import transformations as tf_transformations
#import tf_transformations
import tf2_ros
from tf2_ros import TransformBroadcaster
import re
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from .bridge_node import BridgeNode

#for fake gps
from std_msgs.msg import Float32MultiArray


def mapRanges(value, inMin, inMax, outMin, outMax):
    inSpan = inMax - inMin
    outSpan = outMax - outMin
    valueScaled = float(value - inMin) / float(inSpan)
    return outMin + (valueScaled * outSpan)



class BlueRov(Node):
    def __init__(self):
        super().__init__("blue_rov_node")
        self.bridge = BridgeNode()  
        
        # Services
        self.arm_service = self.create_service(SetBool, '/bluerov2/arm', self.arm_callback, callback_group=ReentrantCallbackGroup())
        self.disarm_service = self.create_service(SetBool, '/bluerov2/disarm', self.disarm_callback, callback_group=ReentrantCallbackGroup())
        self.set_mode_manual_service = self.create_service(SetBool, '/bluerov2/setmode/manual', self.set_mode_manual_callback, callback_group=ReentrantCallbackGroup())
        self.set_mode_alt_hold_service = self.create_service(SetBool, '/bluerov2/setmode/alt_hold', self.set_mode_alt_hold_callback, callback_group=ReentrantCallbackGroup())
        self.set_mode_guided_service = self.create_service(SetBool, '/bluerov2/setmode/guided', self.set_mode_guided_callback, callback_group=ReentrantCallbackGroup())
        self.set_mode_stabilize_service = self.create_service(SetBool, '/bluerov2/setmode/stabilize', self.set_mode_stabilize_callback, callback_group=ReentrantCallbackGroup())
        self.set_mode_position_hold_service = self.create_service(SetBool, '/bluerov2/setmode/position_hold', self.set_mode_position_hold_callback, callback_group=ReentrantCallbackGroup())
        self.pilot_switch_service = self.create_service(SetBool, '/pilot/switch', self.pilot_switch_callback, callback_group=ReentrantCallbackGroup())



        self.timer1 = self.create_timer(1.0 / 30, self.bridge.update) # Collect data at 30Hz
        #self.timer2 = self.create_timer(1.0 / 30, self.publish) # Collect data at 30Hz
        self.send_heartbeat_timer = self.create_timer(0.1, self.send_heartbeat_timer_callback)
        
        
        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )


        self.uuv_name = self.declare_parameter('uuv_name', 'bluerov2').get_parameter_value().string_value
        self.nav_frame = self.declare_parameter('nav_frame', 'map').get_parameter_value().string_value
        self.starttime = time.time()
        self.ROV_name = 'bluerov2'
        self.camera=1500
        self.lights=1000
        self.pilot_sensitivity=25
        self.model_base_link = 'base_link'
        self.thrusterRanges = [1100., 1900.] 
        self.thrusterInputRanges = [-3.0, 3.0]     
        self.time_last_heartbeat = self.get_clock().now()
        self.tf_buffer = tf2_ros.Buffer()
        
        
        self.bluerov_armed = False
        self.bluerov_mode = "no messages"
        self.set_depth = 0.0
        

        # Publishers with custom QoS and unique callback groups
        self.pub_add_wp = self.create_publisher(PoseStamped, f'{self.uuv_name}/navigation/global/add/wp', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pub_goto_wp = self.create_publisher(PoseStamped, f'{self.uuv_name}/navigation/global/goto', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pub_topside_heartbeat = self.create_publisher(Bool, f'{self.uuv_name}/topside_heartbeat', self.custom_qos, callback_group=ReentrantCallbackGroup())
        
        self.odometry_publisher = self.create_publisher(Odometry, f'{self.uuv_name}/odometry', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.battery_publisher = self.create_publisher(BatteryState, f'{self.uuv_name}/battery', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.state_publisher = self.create_publisher(String, f'{self.uuv_name}/state', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.servo_output_publisher = self.create_publisher(UInt16MultiArray, f'/servo_output', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.raw_publisher = self.create_publisher(String, f'{self.uuv_name}/raw', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.imu_data_publisher = self.create_publisher(Imu, f'{self.uuv_name}/imu/data', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.depth_publisher = self.create_publisher(Odometry, f'{self.uuv_name}/depth', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.altitude_publisher = self.create_publisher(PointStamped, f'{self.uuv_name}/altitude', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.bottle_pressure_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/bottle_pressure', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.leak_detection_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/leak_detection', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_1_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_1', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_2_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_2', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_3_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_3', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_4_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_4', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_5_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_5', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_6_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_6', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_7_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_7', self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.pwm_8_publisher = self.create_publisher(UInt16, f'{self.uuv_name}/pwm_8', self.custom_qos, callback_group=ReentrantCallbackGroup())


        self.timer3 = self.create_timer(1.0 / 30, self._create_odometry_msg)  
        self.timer4 = self.create_timer(1.0 / 30, self._create_battery_msg)  
        self.timer5 = self.create_timer(1.0 / 30, self._create_ROV_state)  
        self.timer6 = self.create_timer(1.0 / 30, self._create_servo_msg)
        self.timer7 = self.create_timer(1.0 / 30, self._create_raw_msg)  
        self.timer8 = self.create_timer(1.0 / 30, self._create_imu_msg)  
        self.timer9 = self.create_timer(1.0 / 30, self._create_depth_msg)  
        self.timer10 = self.create_timer(1.0 / 30, self._create_altitude_msg)  
        self.timer13 = self.create_timer(1.0 / 30, self._create_pwm_msg_1)
        self.timer14 = self.create_timer(1.0 / 30, self._create_pwm_msg_2)  
        self.timer15 = self.create_timer(1.0 / 30, self._create_pwm_msg_3)  
        self.timer16 = self.create_timer(1.0 / 30, self._create_pwm_msg_4)  
        self.timer17 = self.create_timer(1.0 / 30, self._create_pwm_msg_5)  
        self.timer18 = self.create_timer(1.0 / 30, self._create_pwm_msg_6)  
        self.timer19 = self.create_timer(1.0 / 30, self._create_pwm_msg_7)  
        self.timer20 = self.create_timer(1.0 / 30, self._create_pwm_msg_8)  
        
     #   self.timer11 = self.create_timer(1.0 / 30, self._create_bottle_pressure_msg)  
      #  self.timer12 = self.create_timer(1.0 / 30, self._create_leak_msg)  
        
        ############
        # Subscriptions
        ############

        # Subscribers
        self.create_subscription(Twist, f'{self.uuv_name}/cmd_vel', self._cmd_vel_callback, self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.create_subscription(String, f'{self.uuv_name}/state', self.handle_bluerov_status, self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.create_subscription(Float32, f'{self.uuv_name}/set_depth', self.handle_set_depth, self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.create_subscription(Bool, '/test_bool_topic', self.test_callback, self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.create_subscription(PoseStamped, f'{self.uuv_name}/attitude_request', self.set_attitude_callback, self.custom_qos, callback_group=ReentrantCallbackGroup())
        self.create_subscription(PoseStamped, f'{self.uuv_name}/position_request', self.set_position_callback, self.custom_qos, callback_group=ReentrantCallbackGroup())
        
      #  self.subscription = self.create_subscription(Float32MultiArray, '/aruco_positions_xy', self.fake_gps_callback, 10, callback_group=ReentrantCallbackGroup())

        self.sub_topics = [
            [self._setpoint_velocity_cmd_vel_callback, "/setpoint_velocity/cmd_vel", TwistStamped,1,],
            [self._set_servo_callback, "/servo{}/set_pwm",UInt16,1, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],],  
            [self._set_rc_channel_callback, "/rc_channel{}/set_pwm", UInt16, 1, [1, 2, 3, 4, 5, 6, 7, 8],],
            [self._set_mode_callback, "/mode/set", String, 1],
            [self._arm_callback, "/arm", Bool, 1],
            [self._handle_topside_heartbeat_callback, "/topside_heartbeat", Bool, 1],
        ]


        for topic in self.sub_topics:
            if len(topic) <= 4:
                callback, topic_name, msg, queue = topic
                self.create_subscription(
                    msg, topic_name, callback, self.custom_qos,
                    callback_group=ReentrantCallbackGroup()
                )
            else:
                callback, topic_name, msg, queue, arg = topic
                for name in arg:
                    self.create_subscription(
                        msg, topic_name.format(name), callback, self.custom_qos,
                        callback_group=ReentrantCallbackGroup()
                    )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


    ############################################
    ########### NODE METHODS ###################
    ############################################

    def test_callback(self, msg):
        if msg.data == True:
            self.get_logger().info('Attitude callback activated')
    
    def set_attitude_callback(self, msg):
       
        thrust = 0.5  # Example thrust value

        # Extract quaternion from the Pose message
        reordered_quaternion = np.array([
            msg.pose.orientation.w,  
            msg.pose.orientation.x,  
            msg.pose.orientation.y,  
            msg.pose.orientation.z   
        ])

        self.bridge.set_attitude_target(
            [
                reordered_quaternion,
                None,
                None,
                0.1,
                thrust,
            ]
        )
             
    def set_position_callback(self, msg):
        self.get_logger().info('position callback activated')
        
        # Extract position and orientation from the message
        x, y, z = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        quaternion = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )

        # Convert quaternion to Euler angles
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        vx = vy = vz = 0.0
        ax = ay = az = None
        vyaw = 0.0 
        force_thruster = 0  
        params = [x, y, z, vx, vy, vz, ax, ay, az, force_thruster, -1 * yaw, vyaw]
        self.bridge.set_position_target_local_ned(params)


    def fake_gps_callback(self, msg):
        # Parse X and Y from the Float32MultiArray
        if len(msg.data) >= 3:  # Check if the message contains at least three elements
            x = msg.data[1]  # X coordinate
            y = msg.data[2]  # Y coordinate
            #lat, lon = self.xy_to_gps(x, y)
            #self.get_logger().info(f'Received XY ({x}, {y}); Converted to lat {lat} lon {lon}')
            self.get_logger().info(f'Received XY ({x}, {y})')
            self.bridge.send_global_vision_position_estimate(x, y)
        else:
            self.get_logger().error('Received data does not contain enough elements.')


    def arm_callback(self, request, response):
        self.bridge.arm_throttle(True)
        response.success = True
        response.message = 'ARM command received'
        return response

    def disarm_callback(self, request, response):
        self.bridge.arm_throttle(False)
        response.success = True
        response.message = 'DISARM command received'
        return response

    def set_mode_manual_callback(self, request, response):
        self.bridge.set_mode("manual")
        response.success = True
        response.message = 'Set mode to MANUAL'
        return response


    def set_mode_alt_hold_callback(self, request, response):
        self.bridge.set_alt_hold_mode()
        response.success = True
        response.message = 'Set mode to ALT_HOLD'
        return response
    
    def set_mode_guided_callback(self, request, response):
        #self.bridge.set_guided_mode()
        self.bridge.set_guided_mode_hold_position_local()
        response.success = True
        response.message = 'Set mode to GUIDED MODE'
        return response

    def set_mode_stabilize_callback(self, request, response):
        self.bridge.set_stabilize_mode()
        response.success = True
        response.message = 'Set mode to STABILIZE'
        return response

    def set_mode_position_hold_callback(self, request, response):
        response.success = True
        response.message = 'Set mode to POSITION_HOLD'
        return response

    def pilot_switch_callback(self, request, response):
        response.success = True
        response.message = 'Pilot switch toggled'
        return response

    def heartbeat_timer_callback(self):
        msg = Bool()
        msg.data = True
        self.pub_topside_heartbeat.publish(msg)

    def handle_bluerov_status(self, msg):
        try:
            status = json.loads(msg.data)
            self.bluerov_armed = status['arm'] == True
            self.bluerov_mode = status['mode']
        except:
            None
            
    def handle_set_depth(self, msg):
        self.set_depth = msg.data
        self.get_logger().info(f"Depth Set To: {self.set_depth}")


    def __del__(self):
        self.bridge.arm_throttle(False)

    @staticmethod
    def _callback_from_topic(topic):

        return topic.replace("/", "_") + "_callback"



    def _set_servo_callback(self, msg, topic):

        paths = topic.split("/")
        servo_id = None
        for path in paths:
            if "servo" in path:
                servo_id = int(re.search("[0-9]", path).group(0)) + 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_servo_pwm(servo_id, msg.data)

    def _set_rc_channel_callback(self, msg, topic):

        paths = topic.split("/")
        channel_id = None
        for path in paths:
            if "rc_channel" in path:
                channel_id = int(re.search("[0-9]", path).group(0)) - 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_rc_channel_pwm(channel_id, msg.data)

    def mapRanges(self, value):
        return np.clip(
            mapRanges(
                value,
                self.thrusterInputRanges[0],
                self.thrusterInputRanges[1],
                self.thrusterRanges[0],
                self.thrusterRanges[1],
            ),
            self.thrusterRanges[0],
            self.thrusterRanges[1],
        )

    def _handle_Arm(self, request, response):
        response.success = True
        response.message = "Armed"
        return response

    def _handle_Disarm(self, request, response):
        response.success = True
        response.message = "Disarmed"
        return response

    def _setModeManual(self, request, response):
        self.bridge.set_mode("manual")
        response.success = True
        response.message = "Mode set to Manual"
        return response

    def _setModeAltHold(self, request, response):
        self.bridge.set_mode("alt_hold")
        response.success = True
        response.message = "Mode set to AltHold"
        return response

    def _setModeStabalize(self, request, response):
        self.bridge.set_mode("stabilize")
        response.success = True
        response.message = "Mode set to Stabilize"
        return response

    def _setModePosition(self, request, response):
        self.bridge.set_mode("guided")
        response.success = True
        response.message = "Mode set to Position (Guided)"
        return response

    def _setModePositionHold(self, request, response):
        self.bridge.set_mode("poshold")
        response.success = True
        response.message = "Mode set to Position Hold"
        return response

    def _pilot_callback(self, msg):

        override = []
        override.append(self.mapRanges(msg.values[4]))  # Pitch
        override.append(self.mapRanges(msg.values[3]))  # Roll
        override.append(self.mapRanges(msg.values[2]))  # throttle
        override.append(self.mapRanges(msg.values[5]))  # Yaw
        override.append(self.mapRanges(msg.values[0]))  # Forward
        override.append(self.mapRanges(msg.values[1]))  # Lateral
        override.append(self.lights)  # light strength
        override.append(self.camera)
        self.bridge.set_rc_channels_pwm(override)

    def _cmd_vel_callback(self, msg):
        override = []
        override.append(self.mapRanges(msg.angular.y))  # Pitch
        override.append(self.mapRanges(msg.angular.x))  # Roll
        override.append(self.mapRanges(msg.linear.z))  # throttle
        override.append(3000 - self.mapRanges(msg.angular.z))  # Yaw
        override.append(self.mapRanges(msg.linear.x))  # Forward
        override.append(3000 - self.mapRanges(msg.linear.y))  # Lateral
        override.append(65535)  # light strength
        override.append(65535)  # camera servo tilt
        self.bridge.set_rc_channels_pwm(override)



    def _set_attitude_callback(self, msg):
        
        thrust = 0.5  

        # Extract quaternion from the Pose message
        quaternion = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        new_quaternion = tf_transformations.quaternion_from_euler(0, 0, -1 * yaw)

        # Set the attitude target
        self.bridge.set_attitude_target(
            [
                new_quaternion[3],
                new_quaternion[0],
                new_quaternion[1],
                new_quaternion[2],
                None,
                None,
                0.1,
                thrust,
            ]
        )

    def _handle_topside_heartbeat_callback(self, _):
        """Topside heartbeat"""
        self.time_last_heartbeat = self.get_clock().now()
        
    def send_heartbeat_timer_callback(self):
        self.bridge.send_heartbeat()

    def _heartbeat_timer_callback(self):
        if (self.get_clock().now() - self.time_last_heartbeat).nanoseconds > 5e9:
            self.bridge.arm_throttle(False)
        

    def _set_mode_callback(self, msg, _):

        self.bridge.set_mode(msg.data)

    def _arm_callback(self, msg, _):

        self.bridge.arm_throttle(msg.data)

    def _setpoint_velocity_cmd_vel_callback(self, msg, _):

        params = [
            None,
            None,
            None,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            None,
            None,
            None,
            None,
            None,
        ]
        self.bridge.set_position_target_local_ned(params)

        # http://mavlink.org/messages/common#SET_ATTITUDE_TARGET
        params = [
            None,
            None,
            None,
            None,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
            None,
        ]
        self.bridge.set_attitude_target(params)

    def _create_header(self, msg):

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.model_base_link
    
    def transform_vector3(self, target_frame, source_frame, vector):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time().to_msg())
            return tf2_geometry_msgs.do_transform_vector3(vector, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform failed: {e}")

            
    def quaternion_from_euler(self, roll, pitch, yaw):
        q = tf2_ros.Quaternion()
        q.setRPY(roll, pitch, yaw)
        return [q.x, q.y, q.z, q.w]
    
    
    def set_data(self, path, value=None, pub=None):
        keys = path.split('/')[1:]
        current_level = self.data
        for part in keys:
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]

        if value is not None and 'pub' in current_level:
            try:
                current_level['pub'].publish(value)
            except Exception as error:
                self.get_logger().error(f'Error publishing to {path}: {error}')

        if pub is not None:
            current_level.update({'pub': pub})
    
    def _create_odometry_msg(self):
        data = self.bridge.get_data()
        print("------------------------------------------")
        print(data)
        if "ATTITUDE" not in data or "LOCAL_POSITION_NED" not in data:
            self.get_logger().error("Missing ATTITUDE or LOCAL_POSITION_NED data")
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Extract position and velocity data
        local_position_data = data["LOCAL_POSITION_NED"]
        xyz_data = [local_position_data[i] for i in ["x", "y", "z"]]
        vxyz_data = [local_position_data[i] for i in ["vx", "vy", "vz"]]

        # Position data
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = -1 * xyz_data[1]  # Inverting Y and Z for NED to ENU conversion, if necessary
        msg.pose.pose.position.z = -1 * xyz_data[2]

        # Assuming ekf_status_data provides variance data for position and velocity
        ekf_status_data = data["EKF_STATUS_REPORT"]
        pos_horiz_variance = ekf_status_data["pos_horiz_variance"]
        pos_vert_variance = ekf_status_data["pos_vert_variance"]
        velocity_variance = ekf_status_data["velocity_variance"]
        # Example: Adding a placeholder for orientation variance (yaw variance here)
        yaw_variance = ekf_status_data.get("yaw_variance", 1.0)  # Placeholder value, adjust as needed

        # Set pose covariance (6x6 matrix flattened into 1D array, row-major order)
        # Only diagonal elements are set for simplicity; adjust as per your system's specifics
        msg.pose.covariance[0] = pos_horiz_variance  # X variance
        msg.pose.covariance[7] = pos_horiz_variance  # Y variance
        msg.pose.covariance[14] = pos_vert_variance  # Z variance
        msg.pose.covariance[21] = yaw_variance  # Roll variance, if applicable
        msg.pose.covariance[28] = yaw_variance  # Pitch variance, if applicable
        msg.pose.covariance[35] = yaw_variance  # Yaw variance

        # Orientation (converted from roll, pitch, yaw to quaternion)
        attitude_data = data["ATTITUDE"]
        orientation = [attitude_data[i] for i in ["roll", "pitch", "yaw"]]
        orientation[1] = -orientation[1]  # Inverting pitch and yaw if NED to ENU conversion is needed
        orientation[2] = -orientation[2]
        q = tf_transformations.quaternion_from_euler(*orientation)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity data
        msg.twist.twist.linear.x = vxyz_data[0]
        msg.twist.twist.linear.y = -1 * vxyz_data[1]
        msg.twist.twist.linear.z = -1 * vxyz_data[2]

        # Angular velocities
        orientation_speed = [attitude_data[i] for i in ["rollspeed", "pitchspeed", "yawspeed"]]
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = -1 * orientation_speed[1]
        msg.twist.twist.angular.z = -1 * orientation_speed[2]

        for i in range(36):
            msg.twist.covariance[i] = velocity_variance if i in [0, 7, 14] else 0.0  # Adjust as needed

        # Publishing the Odometry message
        self.odometry_publisher.publish(msg)

        # Broadcasting the transformation
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)



    def _create_depth_msg(self):


        if "alt" not in self.bridge.get_data()["VFR_HUD"]:
            raise Exception("no alt data")

        msg = Odometry()

        self._create_header(msg)
        msg.header.frame_id = "odom"
        msg.pose.pose.position.z = self.bridge.get_data()["VFR_HUD"]["alt"]
        msg.pose.covariance = np.diag([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]).flatten()
        self.depth_publisher.publish(msg)

    def _create_altitude_msg(self):

        data = self.bridge.get_data()
        if "RANGEFINDER" not in data:
            raise Exception("no RANGEFINDER data")
        range_finder_data = data["RANGEFINDER"]
        altitude = range_finder_data["distance"]

        msg = PointStamped()

        self._create_header(msg)
        msg.header.frame_id = "altitude_link"
        msg.point.z = altitude
        self.altitude_publisher.publish(msg)

    def _create_bottle_pressure_msg(self):
 

        if "press_abs" not in self.bridge.get_data()["SCALED_PRESSURE"]:
            raise Exception("no bottle pressure data")

        msg = UInt16()

        msg.data = self.bridge.get_data()["SCALED_PRESSURE"]["press_abs"]
        self.bottle_pressure_publisher.publish(msg)

    def _create_leak_msg(self):


        if "severity" not in self.bridge.get_data()["STATUSTEXT"]:
            raise Exception("no leak detection data")

        msg = UInt16()
        data = self.bridge.get_data()["STATUSTEXT"]["severity"]
        msg = self.bridge.get_data()["STATUSTEXT"]["text"]
        if data == 2 and msg == "Leak Detected":
            msg.data = 2
        else:
            msg.data = 0
        self.leak_detection_publisher.publish(msg)

    def _create_pwm_msg_1(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo1_raw"]
        msg.data = pwm
        self.pwm_1_publisher.publish(msg)
        

    def _create_pwm_msg_2(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo2_raw"]
        msg.data = pwm
        self.pwm_2_publisher.publish(msg)

    def _create_pwm_msg_3(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo3_raw"]
        msg.data = pwm
        self.pwm_3_publisher.publish(msg)

    def _create_pwm_msg_4(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo4_raw"]
        msg.data = pwm
        self.pwm_4_publisher.publish(msg)

    def _create_pwm_msg_5(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo5_raw"]
        msg.data = pwm
        self.pwm_5_publisher.publish(msg)

    def _create_pwm_msg_6(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo6_raw"]
        msg.data = pwm
        self.pwm_6_publisher.publish(msg)

    def _create_pwm_msg_7(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo7_raw"]
        msg.data = pwm
        self.pwm_7_publisher.publish(msg)

    def _create_pwm_msg_8(self):
        data = self.bridge.get_data()
        if "SERVO_OUTPUT_RAW" not in data:
            raise Exception("no servo data")
        msg = UInt16()
        pwm = data["SERVO_OUTPUT_RAW"]["servo8_raw"]
        msg.data = pwm
        self.pwm_8_publisher.publish(msg)

    def qNED2ENU(self, quat):
        iquat = [quat.x, quat.y, quat.z, quat.w]
        a = np.array([0.707, 0.707, 0, 0])
        b = np.array(
            [
                [
                    [iquat[3], -iquat[2], iquat[1], -iquat[0]],
                    [iquat[2], iquat[3], -iquat[0], -iquat[1]],
                    [-iquat[1], iquat[0], iquat[3], -iquat[2]],
                    [iquat[0], iquat[1], iquat[2], iquat[3]],
                ]
            ]
        )

        c = np.array(
            [
                [
                    [0, 0, -0.707, 0.707],
                    [0, 0, 0.707, 0.707],
                    [0.707, -0.707, 0, 0],
                    [-0.707, -0.707, 0, 0],
                ]
            ]
        )
        q = (a.dot(b)).dot(c)[0][0]
        return Quaternion(-q[0], q[1], q[2], q[3])

       
    def _create_imu_msg(self):

        data = self.bridge.get_data()

        if "ATTITUDE" not in data:
            self.get_logger().error("No ATTITUDE data")
            return

        if "RAW_IMU" not in data:
            self.get_logger().error("No RAW_IMU data")
            return

        imu_data = data["RAW_IMU"]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Adjust scaling factors if needed based on your IMU datasheet
        msg.linear_acceleration.x = imu_data['xacc'] / 100.0
        msg.linear_acceleration.y = imu_data['yacc'] / 100.0
        msg.linear_acceleration.z = imu_data['zacc'] / 100.0
        msg.linear_acceleration_covariance = np.diag([6.235e-4, 7.026e-4, 6.218e-4]).flatten()

        msg.angular_velocity.x = imu_data['xgyro'] / 1000.0
        msg.angular_velocity.y = imu_data['ygyro'] / 1000.0
        msg.angular_velocity.z = imu_data['zgyro'] / 1000.0
        msg.angular_velocity_covariance = np.diag([0.025, 0.025, 0.025]).flatten()

        # Process orientation data from ATTITUDE
        attitude_data = data["ATTITUDE"]
        orientation = [
            attitude_data["roll"],
            -attitude_data["pitch"],
            -attitude_data["yaw"],
        ]
        quaternion = tf_transformations.quaternion_from_euler(*orientation)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        msg.orientation_covariance = np.diag([1.294e-5, 1.324e-5, 2.012e-5]).flatten()

        self.imu_data_publisher.publish(msg)

    def _create_battery_msg(self):
  

        if "SYS_STATUS" not in self.bridge.get_data():
            raise Exception("no SYS_STATUS data")

        if "BATTERY_STATUS" not in self.bridge.get_data():
            raise Exception("no BATTERY_STATUS data")

        bat = BatteryState()
        self._create_header(bat)

        bat.voltage = self.bridge.get_data()["SYS_STATUS"]["voltage_battery"] / 1000
        if bat.voltage < 12.5:
            self.get_logger().warn("Battery is LOW: {}".format(bat.voltage))
        bat.current = self.bridge.get_data()["SYS_STATUS"]["current_battery"] / 100
        bat.percentage = self.bridge.get_data()["BATTERY_STATUS"]["battery_remaining"] / 100
        self.battery_publisher.publish(bat)

    def _create_servo_msg(self):
   
        if "SERVO_OUTPUT_RAW" not in self.bridge.get_data():
            raise Exception("no SYS_STATUS data")

        output = UInt16MultiArray()
        data = self.bridge.get_data()
        servo_data = data["SERVO_OUTPUT_RAW"]
        output.data.append(servo_data["servo1_raw"])
        output.data.append(servo_data["servo2_raw"])
        output.data.append(servo_data["servo3_raw"])
        output.data.append(servo_data["servo4_raw"])
        output.data.append(servo_data["servo5_raw"])
        output.data.append(servo_data["servo6_raw"])
        output.data.append(servo_data["servo7_raw"])
        output.data.append(servo_data["servo8_raw"])

        self.servo_output_publisher.publish(output)

    def _create_raw_msg(self):
        s = self.bridge.get_data()

        string = String()
        string.data = str(json.dumps(s, ensure_ascii=False))

        self.raw_publisher.publish(string)

    def _create_ROV_state(self):
  
        if "SERVO_OUTPUT_RAW" not in self.bridge.get_data():
            raise Exception("no SERVO_OUTPUT_RAW data")

        if "HEARTBEAT" not in self.bridge.get_data():
            raise Exception("no HEARTBEAT data")

        servo_output_raw_msg = self.bridge.get_data()["SERVO_OUTPUT_RAW"]
        servo_output_raw = [
            servo_output_raw_msg["servo{}_raw".format(i + 1)] for i in range(8)
        ]
        rc_channel_msg = self.bridge.get_data()["RC_CHANNELS"]
        rc_channel = [rc_channel_msg["chan{}_raw".format(i + 1)] for i in range(9)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(8)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle / 400
            else:
                throttle = throttle / 500

        light_on = (rc_channel[6] - 1100) / 8
        # need to check
        camera_angle = self.camera - 1500

        # Create angle from pwm
        camera_angle = 45 * camera_angle / 400

        base_mode = self.bridge.get_data()["HEARTBEAT"]["base_mode"]
        custom_mode = self.bridge.get_data()["HEARTBEAT"]["custom_mode"]

        mode, arm = self.bridge.decode_mode(base_mode, custom_mode)

        state = {
            "Gain": self.pilot_sensitivity,
            "motor": motor_throttle,
            "light": light_on,
            "camera_angle": camera_angle,
            "mode": mode,
            "arm": arm,
        }

        string = String()
        string.data = str(json.dumps(state, ensure_ascii=False))

        self.state_publisher.publish(string)
        
        
    def timer_callback(self):
        pass


    
    def publish(self):
        current_time = time.time()
        for create_func, topic, _, _ in self.pub_topics:
            if current_time - self.mavlink_msg_available.get(topic, 0) > 1:
                try:
                    create_func()  
                    self.mavlink_msg_available[topic] = current_time  
                except Exception as e:
                    print(f"Error publishing to {topic}: {e}")


    
def main(args=None):
    rclpy.init(args=args)
    blue_rov_node = BlueRov()
    executor = MultiThreadedExecutor()
    executor.add_node(blue_rov_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        blue_rov_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    
    
