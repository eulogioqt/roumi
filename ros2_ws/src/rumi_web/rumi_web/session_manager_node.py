import os
import json

import rclpy
from rclpy.node import Node

from rumi_msgs.msg import SessionMessage
from rumi_msgs.srv import GetString, SetSessionParams

from .database.system_database import SystemDatabase
from .database.session_manager import SessionManager


class SessionManagerNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.db_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "database/system.db"))
        self.db = SystemDatabase(self.db_path)
        self.sessions = SessionManager(self.db, timeout_seconds=15.0, time_between_detections=0.0)

        self.session_sub = self.create_subscription(SessionMessage, 'rumi/sessions/process', self.session_callback, 10)

        self.get_sessions_serv = self.create_service(GetString, 'rumi/sessions/get', self.get_sessions_service)
        self.set_params_serv = self.create_service(SetSessionParams, 'rumi/sessions/set_params', self.set_session_params_service)

        self.get_logger().info("Session Manager Node initializated succesfully")
        self.create_timer(10.0, self.sessions.check_timeouts)

    def set_session_params_service(self, request, response):
        self.sessions.timeout_seconds = request.timeout_seconds
        self.sessions.time_between_detections = request.time_between_detections

        response.success = True
        response.message = (
            f"Parámetros actualizados: timeout_seconds={request.timeout_seconds}, "
            f"time_between_detections={request.time_between_detections}"
        )
        self.get_logger().info(response.message)

        return response

    def session_callback(self, msg):
        self.get_logger().info(f"Nuevo procesamiento: ({str(msg.faceprint_id)}, {str(msg.detection_score)}, {str(msg.classification_score)})")
        self.sessions.process_detection(str(msg.faceprint_id), float(msg.detection_score), float(msg.classification_score))

    def get_sessions_service(self, request, response):
        args = request.args

        if args:
            args = json.loads(args)

            id = args.get("id", None)
            faceprint_id = args.get("faceprint_id", None)
            if id is not None:
                item = self.db.get_session_by_id(id)
                response.text = json.dumps(item)
            elif faceprint_id is not None:
                items = self.db.get_sessions_by_faceprint_id(faceprint_id)
                response.text = json.dumps(items)
        else:
            items = self.db.get_all_sessions()
            response.text = json.dumps(items)

        return response

def main(args=None):
    rclpy.init(args=args)

    session_manager_node = SessionManagerNode()
    rclpy.spin(session_manager_node)

    rclpy.shutdown()