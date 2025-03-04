#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sancho_msgs.msg import PersonsPoses, PersonPose
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

from .movenet_utils import load_model, run_inference_on_image, process_detections

class MoveNetInferenceNode(Node):
    def __init__(self):
        super().__init__('movenet_inference')
        self.get_logger().info("Iniciando nodo de inferencia MoveNet...")

        # Parámetros configurables
        self.declare_parameter('mirror', False)
        self.declare_parameter('input_size', 256)
        self.declare_parameter('keypoint_score_threshold', 0.4)
        self.declare_parameter('min_keypoints', 9)
        self.declare_parameter('model_url', "https://tfhub.dev/google/movenet/multipose/lightning/1")

        self.mirror = self.get_parameter('mirror').value
        self.input_size = self.get_parameter('input_size').value
        self.keypoint_score_threshold = self.get_parameter('keypoint_score_threshold').value
        self.min_keypoints = self.get_parameter('min_keypoints').value
        self.model_url = self.get_parameter('model_url').value

        # Cargar modelo MoveNet
        self.get_logger().info("Cargando modelo MoveNet...")
        try:
            self.model = load_model(self.model_url)
            self.get_logger().info("Modelo cargado exitosamente.")
        except Exception as e:
            self.get_logger().error(f"Error al cargar el modelo: {e}")
            raise e

        self.bridge = CvBridge()

        # Suscriptor a la imagen de color
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)

        # Publicador para detecciones "raw"
        self.detections_pub = self.create_publisher(PersonsPoses, '/movenet/raw_detections', 10)

    def image_callback(self, color_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        if self.mirror:
            color_image = cv.flip(color_image, 1)

        keypoints_list, scores_list = run_inference_on_image(color_image, self.input_size, self.model)
        valid_detections = process_detections(keypoints_list, scores_list,
                                              self.keypoint_score_threshold, self.min_keypoints)

        detections_msg = PersonsPoses()
        detections_msg.header = color_msg.header

        person_id = 0
        for keypoints, scores in valid_detections:
            person = PersonPose()
            person.header = color_msg.header
            person.id = person_id
            # Flatten 2D keypoints: [x0, y0, x1, y1, ...]
            kp_flat = []
            for (x, y) in keypoints:
                kp_flat.extend([float(x), float(y)])
            person.keypoints = kp_flat
            person.scores = [float(s) for s in scores]
            # keypoints3d se dejarán en cero y se rellenarán en postprocesamiento.
            person.keypoints3d = [0.0] * 51
            person.avg_depth = 0.0

            detections_msg.persons.append(person)
            person_id += 1

        self.detections_pub.publish(detections_msg)
        self.get_logger().info(f"Detecciones publicadas: {len(valid_detections)}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveNetInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo de inferencia...")
    except Exception as e:
        node.get_logger().error(f"Error inesperado: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
