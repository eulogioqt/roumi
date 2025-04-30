import rclpy

from rclpy.node import Node

from hri_msgs.srv import Detection, Recognition, Training, GetString

from hri_vision.hri_bridge import HRIBridge


class ClientNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.get_faceprint_client = self.create_client_wait(GetString, 'recognition/get_faceprint')
        self.detection_client = self.create_client_wait(Detection, 'detection')
        self.recognition_client = self.create_client_wait(Recognition, 'recognition')
        self.training_client = self.create_client_wait(Training, 'recognition/training')

        self.get_sessions_client = self.create_client_wait(GetString, 'logic/get/sessions')

        self.br = HRIBridge()
        self.get_logger().info("ROS Client Node initializated succesfully")

    def create_client_wait(self, srv_type, srv_name):
        client = self.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{srv_name} service not available, waiting again...')

        return client

    def get_faceprint_request(self, args_msg=""):
        get_faceprint_request = GetString.Request()
        get_faceprint_request.args = args_msg

        future_get_faceprint = self.get_faceprint_client.call_async(get_faceprint_request)
        rclpy.spin_until_future_complete(self, future_get_faceprint)
        result_get_faceprint = future_get_faceprint.result()

        return result_get_faceprint.text

    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg, score_msg):
        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg
        recognition_request.score = score_msg

        future_recognition = self.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self, future_recognition)
        result_recognition = future_recognition.result()

        return (result_recognition.face_aligned, result_recognition.features, result_recognition.classified_id,
                result_recognition.classified_name, result_recognition.distance, result_recognition.pos, 
                result_recognition.face_updated)  

    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg
        training_request.origin = Training.Request.ORIGIN_WEB

        future_training = self.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message.data
    
    def get_sessions_request(self, args_msg=""):
        get_sessions_request = GetString.Request()
        get_sessions_request.args = args_msg

        future_get_sessions = self.get_sessions_client.call_async(get_sessions_request)
        rclpy.spin_until_future_complete(self, future_get_sessions)
        result_get_sessions = future_get_sessions.result()

        return result_get_sessions.text