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
        # No initial value setting required here since the input determines the value

    def send_toggle_request(self, value):
        self.req.data = value
        future = self.client.call_async(self.req)
        self.get_logger().info(f'Sending request to set bool to: {self.req.data}')
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def listen_for_input(client_node):
    print("Press '1' to set the boolean to True, or '2' to set it to False.")
    while True:
        user_input = input("Enter '1' for True or '2' for False: ")
        if user_input == '1':
            client_node.send_toggle_request(True)
        elif user_input == '2':
            client_node.send_toggle_request(False)

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
