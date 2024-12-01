import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        # Client for the first service
        self.client1 = self.create_client(SetBool, '/robot/toggle_bool')
        # Client for the second service
        self.client2 = self.create_client(SetBool, '/robot/toggle_another_bool')
        self.wait_for_services()

        self.req1 = SetBool.Request()
        self.req2 = SetBool.Request()

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

def listen_for_input(client_node):
    print("Press '1' or '2' for the first service, '3' or '4' for the second service.")
    while True:
        user_input = input("Enter key: ")
        if user_input == '1':
            client_node.send_request(client_node.client1, client_node.req1, True)
        elif user_input == '2':
            client_node.send_request(client_node.client1, client_node.req1, False)
        elif user_input == '3':
            client_node.send_request(client_node.client2, client_node.req2, True)
        elif user_input == '4':
            client_node.send_request(client_node.client2, client_node.req2, False)

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
