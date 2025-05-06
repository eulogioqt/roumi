import rclpy

from rclpy.node import Node

from rumi_msgs.srv import GetString


class ClientSessionNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.get_sessions_client = self.create_client_wait(GetString, 'rumi/sessions/get')

        self.get_logger().info("ROS Client Node initializated succesfully")

    def create_client_wait(self, srv_type, srv_name):
        client = self.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{srv_name} service not available, waiting again...')

        return client

    def get_sessions_request(self, args_msg=""):
        get_sessions_request = GetString.Request()
        get_sessions_request.args = args_msg

        future_get_sessions = self.get_sessions_client.call_async(get_sessions_request)
        rclpy.spin_until_future_complete(self, future_get_sessions)
        result_get_sessions = future_get_sessions.result()

        return result_get_sessions.text