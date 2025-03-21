import rclpy
import json
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from human_face_recognition_msgs.srv import Recognition, Training, GetString
import cv2
from .hri_bridge import HRIBridge
from .aligners.aligner_dlib import align_face
from .encoders.encoder_facenet import encode_face
from .classifiers.complex_classifier import ComplexClassifier

class HumanFaceRecognizer(Node):

    def __init__(self, use_database = False):
        """Initializes the recognizer node.

        Args:
            use_database (str): If True will use database to load and store data.
        """

        super().__init__("human_face_recognizer")

        self.recognition_count = 0

        show_metrics_param = self.declare_parameter("show_metrics", False)
        self.show_metrics = show_metrics_param.get_parameter_value().bool_value
        self.get_logger().info("Show Metrics: " + str(self.show_metrics))

        self.recognition_service = self.create_service(Recognition, "recognition", self.recognition)
        self.training_service = self.create_service(Training, "recognition/training", self.training)
        self.get_people_service = self.create_service(GetString, "recognition/get_people", self.get_people)
        self.faces_publisher = self.create_publisher(Image, "camera/color/aligned_faces", 10)

        self.classifier = ComplexClassifier(use_database)

        self.training_dispatcher = {
            "refine_class": self.classifier.refine_class,
            "add_features": self.classifier.add_features,
            "add_class": self.classifier.add_class,
            "rename_class": self.classifier.rename_class,
            "delete_class": self.classifier.delete_class
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

        face_aligned = align_face(frame, position)
        
        features = encode_face(face_aligned)
        classified, distance, pos = self.classifier.classify_face(features)

        face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
            self.br.recognizer_to_msg(face_aligned, features, classified, distance, pos)
        )

        response.face_aligned = face_aligned_msg
        response.features = features_msg
        response.classified = classified_msg
        response.distance = distance_msg
        response.pos = pos_msg

        self.faces_publisher.publish(face_aligned_msg)

        self.recognition_count = self.recognition_count + 1
        if self.recognition_count % 30 == 0: # Cada 30 reconocimientos, guarda la información en la base de datos
            self.classifier.save()

        recognition_time = time.time() - start_recognition
        response.recognition_time = recognition_time
        if self.show_metrics:
            self.get_logger().info("Recognition time: " + str(recognition_time))

        return response

    def training(self, request, response):
        """Training service. Takes the command type (training type) and uses the arguments needed to
        perform that kind of training.

        Args:
            request (Training.srv): Command type, and arguments in JSON Object format.

        Returns:
            response (Training.srv): Result. -1 means something went wrong. 0 means everything is okay
                and in case of cmd_type = add_class, also means that the class wasn't already known.
                1 means that the class was already known, and means the same as 0 for cmd_type != add_class.
        """

        cmd_type = request.cmd_type.data
        args = json.loads(request.args.data)

        try:
            function = self.training_dispatcher[cmd_type]
            result, message = function(**args)
        except Exception as e:
            result, message = -1, "Error executing " + cmd_type + ": " + str(e)
            pass

        response.result = result
        response.message = String(data=message)

        return response

    def get_people(self, request, response):
        """Get people service

        Args:
            request (GetString.srv): Empty request (uses service name to know what to do)

        Returns:
            response (String): JSON array with the names of the people in classifier database
        """

        response.text = String(data=self.classifier.get_people())

        return response

def main(args=None):
    rclpy.init(args=args)

    human_face_recognizer = HumanFaceRecognizer()

    rclpy.spin(human_face_recognizer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
