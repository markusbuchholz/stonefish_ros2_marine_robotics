import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.srv1 = self.create_service(SetBool, '/robot/toggle_bool', self.toggle_bool_callback)
        self.srv2 = self.create_service(SetBool, '/robot/toggle_another_bool', self.toggle_another_bool_callback)  # Second service

    def toggle_bool_callback(self, request, response):
        # Dummy logic to toggle a boolean
        response.success = True  # Set the response to indicate success
        response.message = 'Toggled'
        self.get_logger().info(f'Toggled bool: {request.data}')
        return response

    def toggle_another_bool_callback(self, request, response):
        # Logic for the second service, possibly similar to the first
        response.success = True
        response.message = 'Toggled Another Bool'
        self.get_logger().info(f'Toggled another bool: {request.data}')
        return response


def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNode()
    rclpy.spin(server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
