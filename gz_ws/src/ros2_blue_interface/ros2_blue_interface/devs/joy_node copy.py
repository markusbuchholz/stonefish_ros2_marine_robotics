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
#from rclpy.qos import qos_profile_default


LOCAL_MODE = 0
GLOBAL_MODE = 1

class JoystickNode(Node):
    def __init__(self):
        super().__init__("joy_node")
        print (" JOY #####################################################")

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
        self._axes = {'x': 0, 'y': 1, 'z': 2, 'yaw': 3, 'roll': 4, 'pitch': 5, 'light': 6}
        self._axes_gain = {'x': 1.0, 'y': 1.0, 'z': 1.0, 'yaw': 1.0, 'roll': 1.0, 'pitch': 1.0, 'light': 1.0}

        self._deadzone = 0.05
        self.last_cmd_vel_out_was_zero = False

        # Initialize ROS 2 publishers, subscribers, and service clients
        #self.joy_subscription = self.create_subscription(Joy, "/joy", self.handle_joy, qos_profile_sensor_data)

       # self.cmd_vel_publisher = self.create_publisher(Twist, "/bluerov2/cmd_vel", 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/bluerov2/cmd_vel", qos_profile_sensor_data)
        


        cmd_vel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/bluerov2/cmd_vel", cmd_vel_qos)

        
        #self.cmd_vel_sub = self.create_subscription(Twist, '/bluerov2/cmd_vel', self.debug_cmd_vel_callback, qos_profile_sensor_data)

        self.light_level_publisher = self.create_publisher(Float32, "/bluerov2/light/set", 10)
        self.alt_hold_publisher = self.create_publisher(Bool, "/bluerov2/setmode/alt_hold", 10)

        # Initialize client services
        self.arm_service_client = self.create_client(SetBool, "/bluerov2/arm")
        self.disarm_service_client = self.create_client(SetBool, "/bluerov2/disarm")
        self.set_mode_manual_client = self.create_client(SetBool, "/bluerov2/setmode/manual")
        self.set_mode_alt_hold_client = self.create_client(SetBool, "/bluerov2/setmode/alt_hold")
        self.set_mode_stabilize_client = self.create_client(SetBool, "/bluerov2/setmode/stabilize")
        self.set_mode_pos_hold_client = self.create_client(SetBool, "/bluerov2/setmode/position_hold")
        self.pilot_switch = self.create_client(SetBool, "/pilot/switch")

        self.service = self.create_service(SetBool, 'verbose_pid', self.srv_verbose_pid_callback)


        #self.subscription = self.create_subscription(Joy, '/joy', self.handle_joy, 10)
       
        
        # Setup subscriber for Joy messages
        #self.joy_subscriber = self.create_subscription(Joy, '/joy', self.handle_joy, 10)
        self.subscription = self.create_subscription(Joy, "/joy", self.handle_joy, qos_profile=qos_profile_sensor_data)
        #self.subscription = self.create_subscription(Joy, "/joy", self.handle_joy, 1)
        #self.joy_subscriber = self.create_subscription(Joy, "/joy", self.handle_joy, qos_profile=QoSProfile(depth=10))

        
        # Define joystick settings
        self.last_cmd_vel_out_was_zero = True
        self.enabled = True  # Example control flag
        self.acro_btn = 1  # Assuming '1' is the index for your 'acro' button
        
        # Axes and gains mapping
        #self._axes = {'x': 0, 'y': 1, 'z': 2, 'yaw': 3, 'roll': 4, 'pitch': 5}
        #self._axes_gain = {'x': 1.0, 'y': 1.0, 'z': 1.0, 'yaw': 1.0, 'roll': 1.0, 'pitch': 1.0}

        #self.arm_service_client.wait_for_service()
        #self.disarm_service_client.wait_for_service()
        #self.set_mode_manual_client.wait_for_service()
        #self.set_mode_alt_hold_client.wait_for_service()
        #self.set_mode_stabilize_client.wait_for_service()
        #self.set_mode_pos_hold_client.wait_for_service()

        self.btn_action = {
            '0': [self.toggle_manual_btn_pressed, False],  #A
            '2': [self.light_btn_pressed, False],  # X 
            '3': [self.set_mode_manual_pressed, False],  # Y 
            '6': [self.disarm_btn_pressed, False],  # Disarm 
            '7': [self.arm_btn_pressed, False],  # Arm
            '8': [self.pos_hold_btn_pressed, False],  # 
            'na': [self.alt_hold_btn_pressed, False], 
           
        }

    def debug_cmd_vel_callback(self, msg):
        override = []
        override.append(msg.angular.y)  # Pitch
        override.append(msg.angular.x)  # Roll
        override.append(msg.linear.z)  # throttle
        override.append(3000 - msg.angular.z)  # Yaw
        override.append(msg.linear.x)  # Forward
        override.append(3000 - msg.linear.y)  # Lateral
        override.append(65535)  # light strength
        override.append(65535)  # camera servo tilt
        #('debug 2', [1500.0, 1500.0, 1592.1292781829834, 1500.0, 1500.0, 1500.0, 65535, 65535])
        #print("debug 2", override)


    def srv_verbose_pid_callback(self, request, response):
        # ROS 2 uses Python's built-in logging
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

    def handle_joy(self, joy_msg):
        # Handle button press actions
        print ("handle joy ----------------------------------------------------")
        #print ("handle joy ----------------------------------------------------")
        #self.arm_btn_pressed()

        for index, state in enumerate(joy_msg.buttons):
            print ("button:", index, "state :", state  )
            if str(index) in self.btn_action:
                if state==1 and self.btn_action[str(index)][1] == False:
                    self.btn_action[str(index)][0]()
                self.btn_action[str(index)][1] = bool(state)

        # Axis-based actions (like increasing/decreasing light level)
        if joy_msg.axes[6] < 0.0:
            self.decrease_light_level_btn_presses()
        elif joy_msg.axes[6] > 0.0:
            self.increase_light_level_btn_presses()

        if self.enabled:
            linear = Vector3()
            angular = Vector3()

            # Handle joystick movements
            # This section assumes that 'acroBtn' toggles between two control modes
            if joy_msg.buttons[self.acro_btn]:
                print("acro button")
                # Acrobatic mode: use roll and pitch for angular movements
                if abs(joy_msg.axes[self._axes['roll']]) > self._deadzone:
                    angular.x = self._axes_gain['roll'] * joy_msg.axes[self._axes['roll']]
                if abs(joy_msg.axes[self._axes['pitch']]) > self._deadzone:
                    angular.y = self._axes_gain['pitch'] * joy_msg.axes[self._axes['pitch']]
            else:
                # Normal mode: use x and y for linear movements
                if abs(joy_msg.axes[self._axes['x']]) > self._deadzone:
                    linear.x = self._axes_gain['x'] * joy_msg.axes[self._axes['x']]
                if abs(joy_msg.axes[self._axes['y']]) > self._deadzone:
                    linear.y = self._axes_gain['y'] * joy_msg.axes[self._axes['y']]

            # Always use z for linear z-movement and yaw for angular z-rotation
            if abs(joy_msg.axes[self._axes['z']]) > self._deadzone:
                linear.z = self._axes_gain['z'] * joy_msg.axes[self._axes['z']]
            if abs(joy_msg.axes[self._axes['yaw']]) > self._deadzone:
                angular.z = self._axes_gain['yaw'] * joy_msg.axes[self._axes['yaw']]
                
            #print(f"Linear: x={linear.x}, y={linear.y}, z={linear.z}")
            #print(f"Angular: x={angular.x}, y={angular.y}, z={angular.z}")

            # Determine if a command velocity message should be published
            if any([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]):
                self.last_cmd_vel_out_was_zero = False
                #print("publishing ?????????????????????")
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear = linear
                cmd_vel_msg.angular = angular
                self.cmd_vel_publisher.publish(cmd_vel_msg)
            elif self.last_cmd_vel_out_was_zero is False:
                self.last_cmd_vel_out_was_zero = True
                self.cmd_vel_publisher.publish(Twist())  # Publish zero velocities to stop

    def send_service_request(self, service_client, request):
        print("send_service_request")
        """Generic method to send service requests."""
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
                
                
    def light_btn_pressed(self):
        print("light_btn_pressed")
        """Cycle through light levels and publish the selected level."""
        print("light_btn_pressed")
        self.light = (self.light + 1) % len(self.lightLevels)  # Cycle to the next light level
        light_level_msg = Float32(data=self.lightLevels[self.light])  # Prepare the message
        self.light_level_publisher.publish(light_level_msg)  # Publish it
        self.get_logger().info("[VEHICLE_PILOT | JOY] Light level adjusted to: {}".format(self.lightLevels[self.light]))

    def alt_hold_btn_pressed(self):
        print("alt_hold_btn_pressed")
        """Toggle the altitude hold mode and publish the state."""
        if self.enabled:
            self.alt_hold = not self.alt_hold  # Toggle the alt hold state
            alt_hold_msg = Bool(data=self.alt_hold)  # Prepare the message
            self.alt_hold_publisher.publish(alt_hold_msg)  # Publish it
            self.get_logger().info(f"[VEHICLE_PILOT | JOY] Alt hold: {self.alt_hold}")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Trying to activate ALT HOLD failed, vehicle is in autonomous mode")
            

    def set_mode_manual_pressed(self):
        print("set_mode_manual_pressed")
        result = self.send_service_request(self.set_mode_manual_client, SetBool.Request(data=True))
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] is in Manual Mode")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to set Manual Mode")

    def pos_hold_btn_pressed(self):
        print("pos_hold_btn_pressed")
        result = self.send_service_request(self.set_mode_pos_hold_client, SetBool.Request(data=True))
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] is in Poshold Mode")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to set Poshold Mode")        
            
            
    # def local_btn_pressed(self):
    #     if self.enabled:
    #         self.operation_frame = LOCAL_MODE
    #         self.get_logger().info("[VEHICLE_PILOT | JOY] is in LOCAL_MODE")
    #     else:
    #         self.get_logger().warn("[VEHICLE_PILOT | JOY] trying to switch to LOCAL_MODE failed, vehicle is in autonomous mode")

    # def global_btn_pressed(self):
    #     if self.enabled:
    #         self.operation_frame = GLOBAL_MODE
    #         self.get_logger().info("[VEHICLE_PILOT | JOY] is in GLOBAL_MODE]")
    #     else:
    #         self.get_logger().warn("[VEHICLE_PILOT | JOY] trying to switch to GLOBAL_MODE failed, vehicle is in autonomous mode")


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
        print("toggle_manual_btn_pressed")
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
        print("arm_btn_pressed")
        request = SetBool.Request()
        request.data = True  # Request to arm the vehicle
        result = self.send_service_request(self.arm_service_client, request)
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] Vehicle is armed")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to arm the vehicle")

    def disarm_btn_pressed(self):
        print("disarm_btn_pressed!!!!")
        request = SetBool.Request()
        request.data = True  # Request to disarm the vehicle, assuming service expects 'true' to disarm
        result = self.send_service_request(self.disarm_service_client, request)
        print("disarm_btn_pressed - CONFIRMED")
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] Vehicle is disarmed")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to disarm the vehicle")

    def camera_down_btn_pressed(self):
        print("camera_down_btn_pressed")
        self.camera_tilt += self.camera_tilt_gain
        if self.camera_tilt > 2000:
            self.camera_tilt = 2000
        self.camera_tilt_publisher.publish(UInt16(data=self.camera_tilt))

    def camera_up_btn_pressed(self):
        print("camera_up_btn_pressed")
        self.camera_tilt -= self.camera_tilt_gain
        if self.camera_tilt < 1100:
            self.camera_tilt = 1100
        self.camera_tilt_publisher.publish(UInt16(data=self.camera_tilt))

    def increase_light_level_btn_presses(self):
        print("increase_light_level_btn_presses")
        self.light += self.lightGain
        if self.light > 2000:
            self.light = 2000
        self.light_level_publisher.publish(Float32(data=self.light))

    def decrease_light_level_btn_presses(self):
        print("decrease_light_level_btn_presses")
        self.light -= self.lightGain
        if self.light < 1100:
            self.light = 1100
        #self.light_level_publisher.publish(Float32(data=self.light))

    def adjust_light_level(self):
        print("adjust_light_level")
        self.light_level = (self.light_level + 1) % len(self.lightLevels)
        light_msg = Float32()
        light_msg.data = self.lightLevels[self.light_level]
        self.light_level_publisher.publish(light_msg)


def main(args=None):
    rclpy.init(args=args)
    joy_node = JoystickNode()
    rclpy.spin(joy_node)
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()