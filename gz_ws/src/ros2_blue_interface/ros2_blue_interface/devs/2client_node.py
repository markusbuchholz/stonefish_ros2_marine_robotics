import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(SetBool, '/robot/toggle_bool')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()
        self.req.data = True # Initial value to True, but will be toggled with keyboard input

    def toggle_bool(self, data):
        self.req.data = data
        future = self.client.call_async(self.req)
        self.get_logger().info(f'Sending request to toggle bool: {self.req.data}')
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.message}') # Ensure you print `response.message` not `response.response`
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def listen_for_input(client_node):
    print("Press '1' to toggle the boolean on the server.")
    while True:
        user_input = input("Enter '1' to toggle: ")
        if user_input == '1':
            current_state = client_node.req.data
            client_node.toggle_bool(not current_state)

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    input_thread = threading.Thread(target=listen_for_input, args=(client_node,))
    input_thread.daemon = True
    input_thread.start()
    rclpy.spin(client_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
