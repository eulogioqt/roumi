import json
import rclpy

from queue import Queue

from .http_server import HTTPServer
from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer
from .stoppable_node import StoppableNode
from .dynamic_subscribable_node import DynamicSubscribableNode
from .protocol import MessageType, Message, TopicMessage, parse_message
from .r2w_bridge import R2WBridge

from ros2web_msgs.msg import R2WMessage


class ServerNode(DynamicSubscribableNode):

    def __init__(self):
        super().__init__("server")
    
        self.ros_queue = Queue()
        self.broadcast_topics = {}

        self.ros_sub = self.create_subscription(R2WMessage, "ros2web/ros", self.server_callback, 10)
        self.web_pub = self.create_publisher(R2WMessage, "ros2web/web", 10)

        self.r2w_bridge = R2WBridge()
        self.get_logger().info("Server Node initializated succesfully")

    def server_callback(self, msg):
        self.ros_queue.put([msg.key, msg.value])

    def generic_callback(self, topic, name, value):
        self.broadcast_topics[topic] = [name, value]

class Server(StoppableNode):
    
    def __init__(self):
        super().__init__()

        self.node = ServerNode()
        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)
        self.http_server = HTTPServer(port=8173)

    def spin(self):
        if self.node.ros_queue.qsize() > 0: # ROS messages
            [key, value] = self.node.ros_queue.get()

            message = Message(value)
            message_json = message.to_json()

            if key:
                client = self.ws.get_client(key)
                self.ws.send_message(client, message_json)
            else:
                self.ws.broadcast_message(message_json)

        for topic in self.node.broadcast_topics.keys(): # Broadcast topics
            data = self.node.broadcast_topics[topic]
            if data is not None:
                [name, value] = data
                value = self.node.r2w_bridge.any_to_r2w(value)

                message = TopicMessage(topic, name, value)
                message_json = message.to_json()

                self.node.broadcast_topics[topic] = None
                self.ws.broadcast_message(message_json)

    def on_message(self, key, msg):
        type, data = parse_message(msg)
        print(f"Mensaje recibido ({key}): {type}")

        if type == MessageType.MESSAGE:
            value = json.dumps(data) # do this only if is JSON, you should be able to send any kind of data not only json formatted
            self.node.web_pub.publish(R2WMessage(key=key, value=value))

    def on_user_connect(self, key):
        print(f"Cliente conectado ({key}) (Conexiones: {self.ws.get_connection_count()})")
    
    def on_user_disconnect(self, key):
        print(f"Cliente desconectado ({key}) (Conexiones: {self.ws.get_connection_count()})")
    

def main(args=None):
    rclpy.init(args=args)

    server = Server()
    websocket_thread_mixer = WebSocketThreadMixer(server.ws, server, server.http_server)
    websocket_thread_mixer.run()