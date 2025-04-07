#!/usr/bin/env python3

import rclpy
import json
import time
import cv2

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from sensor_msgs.msg import Image
from std_msgs.msg import String
from human_face_recognition_msgs.srv import Recognition, Training, GetString

from .hri_bridge import HRIBridge
from .aligners.aligner_dlib import align_face
from .encoders.encoder_facenet import encode_face
from .classifiers.complex_classifier import ComplexClassifier

class HumanFaceRecognizer(LifecycleNode):
    def __init__(self, use_database=False):
        """Inicializa el nodo de reconocimiento facial."""
        super().__init__("human_face_recognizer")

        self.use_database = use_database
        self.recognition_count = 0

        # Declarar parámetros
        show_metrics_param = self.declare_parameter("show_metrics", False)
        self.show_metrics = show_metrics_param.get_parameter_value().bool_value
        self.get_logger().info(f"Show Metrics: {self.show_metrics}")

        # Recursos ROS inicializados en activate
        self.recognition_service = None
        self.training_service = None
        self.get_people_service = None
        self.faces_publisher = None

        self.classifier = None
        self.br = HRIBridge()

        self.training_dispatcher = {}

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configura los recursos internos."""
        self.get_logger().info("Configurando nodo de reconocimiento facial...")

        try:
            self.classifier = ComplexClassifier(self.use_database)
            self.training_dispatcher = {
                "refine_class": self.classifier.refine_class,
                "add_features": self.classifier.add_features,
                "add_class": self.classifier.add_class,
                "rename_class": self.classifier.rename_class,
                "delete_class": self.classifier.delete_class
            }
        except Exception as e:
            self.get_logger().error(f"Error inicializando classifier: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Crea servicios y publicadores."""
        self.get_logger().info("Activando nodo...")

        try:
            self.recognition_service = self.create_service(Recognition, "recognition", self.recognition_callback)
            self.training_service = self.create_service(Training, "recognition/training", self.training_callback)
            self.get_people_service = self.create_service(GetString, "recognition/get_people", self.get_people_callback)
            self.faces_publisher = self.create_publisher(Image, "camera/color/aligned_faces", 10)
        except Exception as e:
            self.get_logger().error(f"Error creando interfaces ROS: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destruye servicios y publicadores."""
        self.get_logger().info("Desactivando nodo...")

        if self.recognition_service is not None:
            self.destroy_service(self.recognition_service)
            self.recognition_service = None
        if self.training_service is not None:
            self.destroy_service(self.training_service)
            self.training_service = None
        if self.get_people_service is not None:
            self.destroy_service(self.get_people_service)
            self.get_people_service = None
        if self.faces_publisher is not None:
            self.destroy_publisher(self.faces_publisher)
            self.faces_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Limpia estado interno."""
        self.get_logger().info("Limpiando recursos internos...")
        self.classifier = None
        self.training_dispatcher.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Apaga el nodo de forma limpia."""
        self.get_logger().info("Apagando nodo HumanFaceRecognizer...")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Manejo de errores."""
        self.get_logger().error("Error grave en el nodo de reconocimiento facial.")
        return TransitionCallbackReturn.SUCCESS

    def recognition_callback(self, request, response):
        """Servicio de reconocimiento facial."""
        start_time = time.time()

        try:
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

            if self.faces_publisher is not None:
                self.faces_publisher.publish(face_aligned_msg)

            self.recognition_count += 1
            if self.recognition_count % 30 == 0:
                self.classifier.save()

            recognition_time = time.time() - start_time
            response.recognition_time = recognition_time
            if self.show_metrics:
                self.get_logger().info(f"Recognition time: {recognition_time:.3f}s")

        except Exception as e:
            self.get_logger().error(f"Error en reconocimiento: {e}")

        return response

    def training_callback(self, request, response):
        """Servicio de entrenamiento."""
        cmd_type = request.cmd_type.data
        args = json.loads(request.args.data)

        try:
            function = self.training_dispatcher.get(cmd_type)
            if function is None:
                raise ValueError(f"Comando de entrenamiento desconocido: {cmd_type}")

            result, message = function(**args)
        except Exception as e:
            result, message = -1, f"Error ejecutando {cmd_type}: {str(e)}"
            self.get_logger().error(message)

        response.result = result
        response.message = String(data=message)
        return response

    def get_people_callback(self, request, response):
        """Servicio para obtener la lista de personas."""
        try:
            people_json = self.classifier.get_people()
            response.text = String(data=people_json)
        except Exception as e:
            self.get_logger().error(f"Error obteniendo lista de personas: {e}")
            response.text = String(data="[]")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HumanFaceRecognizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado. Cerrando nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
