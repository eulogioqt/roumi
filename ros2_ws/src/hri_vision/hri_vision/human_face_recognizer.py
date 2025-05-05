import json
import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hri_msgs.srv import Recognition, Training, GetString
from ros2web_msgs.msg import R2WMessage

from .hri_bridge import HRIBridge
from .aligners.aligner_dlib import align_face
from .encoders.encoder_facenet import encode_face
from .classifiers.complex_classifier import ComplexClassifier


class HumanFaceRecognizer(Node):

    def __init__(self):
        """Initializes the recognizer node"""

        super().__init__("human_face_recognizer")

        show_metrics_param = self.declare_parameter("show_metrics", False)
        self.show_metrics = show_metrics_param.get_parameter_value().bool_value
        self.get_logger().info(f"Show Metrics: {self.show_metrics}")

        self.recognition_service = self.create_service(Recognition, "recognition", self.recognition)
        self.training_service = self.create_service(Training, "recognition/training", self.training)
        self.get_faceprint_service = self.create_service(GetString, "recognition/get_faceprint", self.get_people)

        self.ros_pub = self.create_publisher(R2WMessage, "ros2web/ros", 10)

        self.classifier = ComplexClassifier()
        self.save_db_timer = self.create_timer(10.0, self.save_data)

        self.training_dispatcher = {
            "refine_class": self.classifier.refine_class,
            "add_features": self.classifier.add_features,
            "add_class": self.classifier.add_class, # y este igual realmente, pensarlo bien porque en vd si es train "nueva clase"

            "rename_class": self.classifier.rename_class, # cambiar estos
            "delete_class": self.classifier.delete_class # porque no son training, simplemente por significado
        }

        self.faceprint_event_map = {
            "add_class": "CREATE",
            "delete_class": "DELETE",
            "rename_class": "UPDATE",
            "add_features": "UPDATE",
        }

        self.br = HRIBridge()

    def recognition(self, request, response):
        """Recognition service

        Args:
            request (Recognition.srv): Frame and a face position

        Returns:
            response (Recognition.srv): Face aligned, features, class predicted, distance (score) and p
                osition of the vector in the class with highest distance (score).
        """

        start_recognition = time.time()

        frame = self.br.imgmsg_to_cv2(request.frame, "bgr8")
        position = [
            request.position.x,
            request.position.y,
            request.position.w,
            request.position.h,
        ]
        score = request.score
        size = math.sqrt(position[2]**2 + position[3]**2)
        
        face_aligned = align_face(frame, position)
        
        features = encode_face(face_aligned)
        faceprint, distance, pos = self.classifier.classify_face(features)
        classified_id = faceprint["id"] if faceprint else None
        classified_name = faceprint["name"] if faceprint else None

        UPPER_BOUND = 0.9
        if faceprint and score >= 1 and distance >= UPPER_BOUND: 
            face_updated = self.classifier.save_face(classified_id, face_aligned, score) 
            if face_updated:
                self.send_faceprint_event("UPDATE", classified_id)
        else:
            face_updated = False

        face_aligned_msg, features_msg, classified_name_msg, distance_msg, pos_msg = (
            self.br.recognizer_to_msg(face_aligned, features, classified_name, distance, pos)
        )   

        response.face_aligned = face_aligned_msg
        response.features = features_msg
        response.classified_id = str(classified_id)
        response.classified_name = classified_name_msg
        response.distance = distance_msg
        response.pos = pos_msg
        response.face_updated = face_updated 

        recognition_time = time.time() - start_recognition
        response.recognition_time = recognition_time
        if self.show_metrics:
            self.get_logger().info("Recognition time: " + str(recognition_time))

        return response

    def training(self, request, response):
        """
        Handles training-related service requests by dispatching them to the corresponding handler
        based on the command type and arguments.

        Args:
            request (Training.srv): Contains the command type and arguments (in JSON format).
            response (Training.srv): Will be filled with the result and a message.

        Returns:
            Training.srv: Response object with result code and message.
                result = -1 → error
                result = 0  → something not totally ok
                result = 1  → success class already existed (only meaningful for cmd_type == "add_class")
        """

        try:
            cmd_type = request.cmd_type.data
            args = json.loads(request.args.data)
            origin = request.origin
        except json.JSONDecodeError as e:
            response.result = -1
            response.message = String(data=f"Invalid JSON: {e}")
            return response

        try:
            function = self.training_dispatcher[cmd_type]
            result, message = function(**args)
        except Exception as e:
            result, message = -1, f"Error executing {cmd_type}: {e}"

        if result >= 0 and "class_name" in args: # Send faceprint event        
            event = self.faceprint_event_map.get(cmd_type)
            if event is not None:
                id = message if cmd_type == "add_class" else args["class_id"]
                if origin != Training.Request.ORIGIN_WEB:
                    self.send_faceprint_event(event, id)

        response.result = result
        response.message = String(data=str(message))

        return response

    def send_faceprint_event(self, event, id):
        message = {
            "type": "FACEPRINT_EVENT",
            "data": {
                "event": event,
                "id": id
            }
        }

        message_json = json.dumps(message)
        self.ros_pub.publish(R2WMessage(value=message_json))

    def get_people(self, request, response):
        args = request.args

        if args:
            args = json.loads(args)
            id = args["id"]

            faceprint = self.classifier.db.get_by_id(id)
            response.text = json.dumps(faceprint)
        else:
            faceprints = self.classifier.db.get_all()
            response.text = json.dumps(faceprints)

        return response

    def save_data(self):
        self.classifier.db.save()

def main(args=None):
    rclpy.init(args=args)

    human_face_recognizer = HumanFaceRecognizer()

    rclpy.spin(human_face_recognizer)
    rclpy.shutdown()
