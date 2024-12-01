from std_msgs.msg import Float32

from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile


LOCAL_MODE = 0
GLOBAL_MODE = 1

class JoystickNode(Node):
    def __init__(self):
        super().__init__("simple_joy_node")

        # Button mappings and states
        self.acroBtn = 1  # B button for acrobatic mode toggle
        self.lightLevels = [0.0, 0.33, 0.66, 1.0]
        self.light = 0
        self.lightGain = 25.0
        self.manualControl = True
        self.altHold = False
        self.operationFrame = LOCAL_MODE
        self.enabled = True

        # Axis configurations
        self._axes = {'x': 0, 'y': 1, 'z': 2, 'yaw': 3, 'roll': 4, 'pitch': 5, 'light': 6}
        self._axes_gain = {'x': 1.0, 'y': 1.0, 'z': 1.0, 'yaw': 1.0, 'roll': 1.0, 'pitch': 1.0, 'light': 1.0}

        self._deadzone = 0.05
        self.last_cmd_vel_out_was_zero = False

        self.subscription = self.create_subscription(Joy, '/joy', self.handle_joy, qos_profile=QoSProfile(depth=10))

        # Initialize client services
        self.arm_service_client = self.create_client(SetBool, '/bluerov2/arm')
        self.disarm_service_client = self.create_client(SetBool, '/bluerov2/disarm')
        self.set_mode_manual_client = self.create_client(SetBool, '/bluerov2/setmode/manual')
        self.set_mode_alt_hold_client = self.create_client(SetBool, '/bluerov2/setmode/alt_hold')
        self.set_mode_stabilize_client = self.create_client(SetBool, '/bluerov2/setmode/stabilize')
        self.set_mode_pos_hold_client = self.create_client(SetBool, '/bluerov2/setmode/position_hold')
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=QoSProfile(depth=10))

        print("WAITING FOR SERVICE CLIENTS")

        self.btn_action = {
            '0': [self.toggle_manual_btn_pressed, True],  # A
            '2': [self.light_btn_pressed, True],  # X 
            '3': [self.set_mode_manual_pressed, True],  # Y 
            '6': [self.disarm_btn_pressed, True],  # Disarm 
            '7': [self.arm_btn_pressed, True],  # Arm
            '8': [self.pos_hold_btn_pressed, True],  # 
            'na': [self.alt_hold_btn_pressed, True], 
        }

        print("SERVICE CLIENTS READY")

    def handle_joy(self, joy_msg):
        # Handle button press actions
        for index, state in enumerate(joy_msg.buttons):
            if str(index) in self.btn_action:
                if state == 1 and self.btn_action[str(index)][1] == False:
                    #await self.btn_action[str(index)][0]()
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
            if joy_msg.buttons[self.acroBtn]:
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

            # Determine if a command velocity message should be published
            if any([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]):
                self.last_cmd_vel_out_was_zero = False
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear = linear
                cmd_vel_msg.angular = angular
                self.cmd_vel_publisher.publish(cmd_vel_msg)
            elif self.last_cmd_vel_out_was_zero is False:
                self.last_cmd_vel_out_was_zero = True
                self.cmd_vel_publisher.publish(Twist())  # Publish zero velocities to stop

    async def send_service_request(self, service_client, request):
        """Generic method to send service requests."""
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
                
    async def light_btn_pressed(self):
        """Cycle through light levels and publish the selected level."""
        self.light = (self.light + 1) % len(self.lightLevels)  # Cycle to the next light level
        light_level_msg = Float32(data=self.lightLevels[self.light])  # Prepare the message
        self.get_logger().info("[VEHICLE_PILOT | JOY] Light level adjusted to: {}".format(self.lightLevels[self.light]))

    async def alt_hold_btn_pressed(self):
        """Toggle the altitude hold mode and publish the state."""
        if self.enabled:
            self.alt_hold = not self.alt_hold  # Toggle the alt hold state
            alt_hold_msg = Bool(data=self.alt_hold)  # Prepare the message
            self.get_logger().info(f"[VEHICLE_PILOT | JOY] Alt hold: {self.alt_hold}")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Trying to activate ALT HOLD failed, vehicle is in autonomous mode")

    async def set_mode_manual_pressed(self):
        result = await self.send_service_request(self.set_mode_manual_client, SetBool.Request(data=True))
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] is in Manual Mode")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to set Manual Mode")

    async def pos_hold_btn_pressed(self):
        result = await self.send_service_request(self.set_mode_pos_hold_client, SetBool.Request(data=True))
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] is in Poshold Mode")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to set Poshold Mode")        

    async def toggle_manual_btn_pressed(self):
        request = SetBool.Request()

        print("[VEHICLE_PILOT | JOY] Vehicle is in")

    async def arm_btn_pressed(self):
        request = SetBool.Request()
        request.data = True  # Request to arm the vehicle
        result = await self.send_service_request(self.arm_service_client, request)
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] Vehicle is armed")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to arm the vehicle")

    async def disarm_btn_pressed(self):
        request = SetBool.Request()
        request.data = True  # Request to disarm the vehicle, assuming service expects 'true' to disarm
        result = await self.send_service_request(self.disarm_service_client, request)
        if result is not None and result.success:
            self.get_logger().info("[VEHICLE_PILOT | JOY] Vehicle is disarmed")
        else:
            self.get_logger().warn("[VEHICLE_PILOT | JOY] Failed to disarm the vehicle")

    async def increase_light_level_btn_presses(self):
        self.light += self.lightGain

    async def decrease_light_level_btn_presses(self):
        self.light -= self.lightGain

def main(args=None):
    rclpy.init(args=args)
    joy_node = JoystickNode()
    rclpy.spin(joy_node)
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

