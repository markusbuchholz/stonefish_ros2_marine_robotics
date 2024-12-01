import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from sensor_msgs.msg import Joy

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        # Dictionary of service topics
        self.service_topics = {
            'arm': '/bluerov2/arm',
            'disarm': '/bluerov2/disarm',
            'set_mode_manual': '/bluerov2/setmode/manual',
            'set_mode_alt_hold': '/bluerov2/setmode/alt_hold',
            'set_mode_stabilize': '/bluerov2/setmode/stabilize',
            'set_mode_position_hold': '/bluerov2/setmode/position_hold',
            'pilot_switch': '/pilot/switch'
        }
        # Initialize service clients, renaming 'clients' to 'service_clients' to avoid attribute error
        self.service_clients = {name: self.create_client(SetBool, topic) for name, topic in self.service_topics.items()}
        # Initialize requests
        self.requests = {name: SetBool.Request() for name in self.service_topics}

        # Wait for all services to be available
        self.wait_for_services()

        # Subscribe to /joy topic
        self.subscription = self.create_subscription(
            Joy, "/joy", self.handle_joy, qos_profile_sensor_data
        )

    def wait_for_services(self):
        ready = False
        while not ready:
            ready = all(client.wait_for_service(timeout_sec=1.0) for client in self.service_clients.values())
            if not ready:
                self.get_logger().info('One or more services not available, waiting again...')

    def send_request(self, service_name, value):
        if service_name in self.requests:
            request = self.requests[service_name]
            request.data = value
            future = self.service_clients[service_name].call_async(request)
            self.get_logger().info(f'Sending request to {service_name}: {value}')
            future.add_done_callback(self.callback)
        else:
            self.get_logger().error(f'Service {service_name} not found')

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Received response: {response.message}')
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def handle_joy(self, msg):
        # Assuming buttons 7 and 8 for "arm/disarm" for demonstration purposes
        if msg.buttons[6] == 1:  # Index for button 7
            self.send_request('arm', True)
        if msg.buttons[7] == 1:  # Index for button 8
            self.send_request('disarm', True)
        # Add additional button handlers as needed

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    rclpy.spin(client_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
