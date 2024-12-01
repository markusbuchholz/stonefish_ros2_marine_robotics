## Jonatan Scharff Willners @2023
## Markus Buchholz @2024 @ROS 2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.qos import ReliabilityPolicy, DurabilityPolicy




LOCAL_MODE = 0
GLOBAL_MODE = 1

class JoystickNode(Node):
    def __init__(self):
        super().__init__("joy_node")

        # Button mappings and states
        self.acroBtn = 1  # B button for acrobatic mode toggle
        self.lightLevels = [0.0, 0.33, 0.66, 1.0]
        self.light = 0
        self.lightGain = 25.0
        self.cameraTilt = 1500
        self.cameraTiltGain = 25.0
        self.manualControl = True
        self.altHold = False
        self.operationFrame = LOCAL_MODE
        self.enabled = True
        self.qos_profile_default = QoSProfile(depth=10)

        # Axis configurations
        self._axes = {'x': 1, 'y': 0, 'z': 4, 'yaw': 3, 'roll': 0, 'pitch': 1}
        #boat
        #self._axes = {'x': 4, 'y': 0, 'z': 4, 'yaw': 3, 'roll': 0, 'pitch': 1}
        self._axes_gain = {'x': 3.0, 'y': 3.0, 'z': 3.0, 'yaw': 3.0, 'roll': -3.0, 'pitch': -3.0}
      
        # Define joystick settings
        self._deadzone = 0.05
        self.last_cmd_vel_out_was_zero = False
        self.enabled = True  
        self.acro_btn = 1  
        self.armed = True
        self.counter = 0
        
        # Define a custom QoS profile
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Setup subscriber for Joy messages
        self.subscription = self.create_subscription(Joy, "/joy", self.handle_joy, qos_profile=qos_profile_sensor_data)

        #Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/bluerov2/cmd_vel", qos_profile=cmd_vel_qos)
        self.light_level_publisher = self.create_publisher(Float32, "/bluerov2/light/set", qos_profile=custom_qos)
        


        # Initialize client services
        self.service_group = ReentrantCallbackGroup()
        self.service_topics = {
            'arm': '/bluerov2/arm',
            'disarm': '/bluerov2/disarm',
            'set_mode_manual': '/bluerov2/setmode/manual',
            'set_mode_alt_hold': '/bluerov2/setmode/alt_hold',
            'set_mode_guided': '/bluerov2/setmode/guided',
            'set_mode_stabilize': '/bluerov2/setmode/stabilize',
            'set_mode_position_hold': '/bluerov2/setmode/position_hold',
            'pilot_switch': '/pilot/switch'
        }
        self.service_clients = {
            name: self.create_client(SetBool, topic, callback_group=self.service_group)
            for name, topic in self.service_topics.items()
        }
        self.requests = {name: SetBool.Request() for name in self.service_topics}
        self.wait_for_services()
        
        self.last_joystick_time = self.get_clock().now()
        self.inactivity_timer = self.create_timer(0.05, self.check_joystick_inactivity) 

    def wait_for_services(self):
        # Wait for all service clients to be available
        for name, client in self.service_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {name} to be available...')

                
    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Received response: {response.message}')
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    def send_request(self, service_name, value):
        if service_name in self.requests:
            request = self.requests[service_name]
            request.data = value
            future = self.service_clients[service_name].call_async(request)
            self.get_logger().info(f'Sending request to {service_name}: {value}')
            future.add_done_callback(self.callback)
        else:
            self.get_logger().error(f'Service {service_name} not found')
        


    def srv_verbose_pid_callback(self, request, response):
        self.get_logger().info('Pilot switch got request: %s' % str(request.data))
        if request.data:
            self.verbose_pid = True
            response.success = True
            response.message = ""
        else:
            self.verbose_pid = False
            response.success = False
            response.message = ""
        return response

    def createStatusMessage(self):
        ctrlMode = "Autonomous" if not self.enabled else "Joystick"
        frame = "Local" if self.operationFrame == "local_mode" else "Global"
        msg = f"ControlMode: {ctrlMode}, Frame: {frame}, AltHold: {self.altHold}, Light: {self.light}, CameraTilt: {self.cameraTilt}"
        self.get_logger().info(msg)



    def check_joystick_inactivity(self):
        elapsed_time = (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9
        if elapsed_time > 0.1:
            self.cmd_vel_publisher.publish(Twist())  # Send zero velocities to stop the robot

    # right axis [4] up/doww
    # left axis [0] left /right

    def handle_joy(self, joy_msg):
        #print(" -- handle joy -- ")
        self.last_joystick_time = self.get_clock().now()

        # Handling button press actions
        if joy_msg.buttons[7] == 1:  # Index for button ARM
            self.send_request('arm', True)
            self.armed = True
        if joy_msg.buttons[6] == 1:  # Index for button DISARM
            self.send_request('disarm', True)
            self.armed = False
        if joy_msg.buttons[0] == 1:  # Index for button A
            self.send_request('set_mode_alt_hold', True)
        if joy_msg.buttons[1] == 1:  # Index for button B
            self.send_request('set_mode_manual', True)
        if joy_msg.buttons[2] == 1:  # Index for button X
            self.send_request('set_mode_guided', True)
        if joy_msg.buttons[3] == 1:  # Index for button Y
            self.send_request('set_mode_stabilize', True)

        # Do not proceed if not armed
        if not self.armed:
            return

        # Handling joystick axis movements
        movement_detected = False
        linear = Vector3()
        angular = Vector3()

        # Handle joystick movements
        if self.enabled:
            # Check each axis for movement and apply deadzone check
            if abs(joy_msg.axes[self._axes['x']]) > self._deadzone:
                linear.x = self._axes_gain['x'] * joy_msg.axes[self._axes['x']]
                movement_detected = True
            if abs(joy_msg.axes[self._axes['y']]) > self._deadzone:
                linear.y = self._axes_gain['y'] * joy_msg.axes[self._axes['y']]
                movement_detected = True
            if abs(joy_msg.axes[self._axes['z']]) > self._deadzone:
                linear.z = self._axes_gain['z'] * joy_msg.axes[self._axes['z']]
                movement_detected = True
            if abs(joy_msg.axes[self._axes['yaw']]) > self._deadzone:
                angular.z = self._axes_gain['yaw'] * joy_msg.axes[self._axes['yaw']]
                movement_detected = True
            if joy_msg.buttons[self.acro_btn]:  # Acrobatic mode
                if abs(joy_msg.axes[self._axes['roll']]) > self._deadzone:
                    angular.x = self._axes_gain['roll'] * joy_msg.axes[self._axes['roll']]
                    movement_detected = True
                if abs(joy_msg.axes[self._axes['pitch']]) > self._deadzone:
                    angular.y = self._axes_gain['pitch'] * joy_msg.axes[self._axes['pitch']]
                    movement_detected = True

            # Publish the cmd_vel message if any movement is detected
            if movement_detected:
                self.last_cmd_vel_out_was_zero = False
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear = linear
                cmd_vel_msg.angular = angular
                self.cmd_vel_publisher.publish(cmd_vel_msg)
            elif not self.last_cmd_vel_out_was_zero:  
                self.last_cmd_vel_out_was_zero = True
                self.cmd_vel_publisher.publish(Twist())  


    def send_service_request(self, service_client, request):
        print("send_service_request")
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
                
                
    def light_btn_pressed(self):
        print("light_btn_pressed")
        print("light_btn_pressed")
        self.light = (self.light + 1) % len(self.lightLevels)  # Cycle to the next light level
        light_level_msg = Float32(data=self.lightLevels[self.light])  # Prepare the message
        self.light_level_publisher.publish(light_level_msg)  # Publish it
        self.get_logger().info("[VEHICLE_PILOT | JOY] Light level adjusted to: {}".format(self.lightLevels[self.light]))

    def alt_hold_btn_pressed(self):
        if self.enabled:
            self.alt_hold = not self.alt_hold  # Toggle the alt hold state
            alt_hold_msg = Bool(data=self.alt_hold)  # Prepare the message
            self.alt_hold_publisher.publish(alt_hold_msg)  # Publish it
            self.get_logger().info(f"[VEHICLE_PILOT | JOY] Alt hold: {self.alt_hold}")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Trying to activate ALT HOLD failed, vehicle is in autonomous mode")
            

    def set_mode_manual_pressed(self):
        result = self.send_service_request(self.set_mode_manual_client, SetBool.Request(data=True))
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] is in Manual Mode")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to set Manual Mode")

    def pos_hold_btn_pressed(self):
        result = self.send_service_request(self.set_mode_pos_hold_client, SetBool.Request(data=True))
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] is in Poshold Mode")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to set Poshold Mode")        
            

    def toggle_hold_position(self):
        print("toggle_hold_position")
        request = SetBool.Request()
        if self.enabled:
            self.clear_wps_publisher.publish(Bool(data=True))
            request.data = True  # Activate station keeping
        else:
            request.data = False  # Deactivate station keeping
        
        result = self.send_service_request(self.pilot_switch, request)
        if result is not None and result.success:
            self.enabled = not self.enabled
            action = "ON" if request.data else "OFF"
            self.get_logger().info(f"[VEHICLE_PILOT | JOY] : Station keeping {action}")
        else:
            self.get_logger().warn(f"[VEHICLE_PILOT | JOY] : Failed to set station keeping {action}")

    def toggle_manual_btn_pressed(self):
        request = SetBool.Request()
        request.data = not self.enabled  # Toggle state
        result = self.send_service_request(self.pilot_switch, request)
        if result is not None and result.success:
            self.enabled = request.data
            mode = "Autonomous Mode" if request.data else "Remote Controlled Mode"
            self.get_logger().info(f"[VEHICLE_PILOT | JOY] Vehicle is in {mode}")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] : Failed to toggle mode")


    def arm_btn_pressed(self):
        request = SetBool.Request()
        request.data = True  # Request to arm the vehicle
        result = self.send_service_request(self.arm_service_client, request)
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] Vehicle is armed")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to arm the vehicle")

    def disarm_btn_pressed(self):
        request = SetBool.Request()
        request.data = True  # Request to disarm the vehicle, assuming service expects 'true' to disarm
        result = self.send_service_request(self.disarm_service_client, request)
        print("disarm_btn_pressed - CONFIRMED")
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] Vehicle is disarmed")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to disarm the vehicle")

    def camera_down_btn_pressed(self):
        self.camera_tilt += self.camera_tilt_gain
        if self.camera_tilt > 2000:
            self.camera_tilt = 2000
        self.camera_tilt_publisher.publish(UInt16(data=self.camera_tilt))

    def camera_up_btn_pressed(self):
        self.camera_tilt -= self.camera_tilt_gain
        if self.camera_tilt < 1100:
            self.camera_tilt = 1100
        self.camera_tilt_publisher.publish(UInt16(data=self.camera_tilt))

    def increase_light_level_btn_presses(self):
        self.light += self.lightGain
        if self.light > 2000:
            self.light = 2000
        self.light_level_publisher.publish(Float32(data=self.light))

    def decrease_light_level_btn_presses(self):
        self.light -= self.lightGain
        if self.light < 1100:
            self.light = 1100
        #self.light_level_publisher.publish(Float32(data=self.light))

    def adjust_light_level(self):
        self.light_level = (self.light_level + 1) % len(self.lightLevels)
        light_msg = Float32()
        light_msg.data = self.lightLevels[self.light_level]
        self.light_level_publisher.publish(light_msg)

    
def main(args=None):
    rclpy.init(args=args)
    joy_node = JoystickNode()
    executor = MultiThreadedExecutor()
    executor.add_node(joy_node)

    try:
        joy_node.get_logger().info('joy_node is running')
        executor.spin()
    except KeyboardInterrupt:
        joy_node.get_logger().info('joy_node is shutting down')
    finally:
        joy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()