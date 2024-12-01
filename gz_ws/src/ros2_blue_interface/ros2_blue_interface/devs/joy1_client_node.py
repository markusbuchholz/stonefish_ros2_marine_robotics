import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from sensor_msgs.msg import Joy

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        # Clients for the services
        self.client1 = self.create_client(SetBool, '/robot/toggle_bool')
        self.client2 = self.create_client(SetBool, '/robot/toggle_another_bool')
        
        self.req1 = SetBool.Request()
        self.req2 = SetBool.Request()
        
        # Wait for both services to be available
        self.wait_for_services()
        
        # Subscribe to /joy topic
        self.subscription = self.create_subscription(
            Joy,
            "/joy",
            self.handle_joy,
            qos_profile_sensor_data
        )

    def wait_for_services(self):
        while not self.client1.wait_for_service(timeout_sec=1.0) or not self.client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('One or more services not available, waiting again...')

    def send_request(self, client, request, value):
        request.data = value
        future = client.call_async(request)
        self.get_logger().info(f'Sending request: {value}')
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def handle_joy(self, msg):
        # Button 7 (index 6) for service 1 True, Button 6 (index 5) for service 1 False
        if msg.buttons[7] == 1:
            self.send_request(self.client1, self.req1, True)
        elif msg.buttons[6] == 1:
            self.send_request(self.client1, self.req1, False)
        
        # Button 3 (index 2) for service 2 True, Button 4 (index 3) for service 2 False
        if msg.buttons[2] == 1:
            self.send_request(self.client2, self.req2, True)
        elif msg.buttons[3] == 1:
            self.send_request(self.client2, self.req2, False)

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    rclpy.spin(client_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
