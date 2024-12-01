import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(SetBool, '/robot/toggle_bool')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()
        self.req.data = True # Initial value

    def toggle_bool(self):
        self.req.data = not self.req.data
        future = self.client.call_async(self.req)
        self.get_logger().info(f'Sending request to toggle bool: {self.req.data}')
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    for _ in range(5):
        client_node.toggle_bool()
    rclpy.spin_once(client_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
