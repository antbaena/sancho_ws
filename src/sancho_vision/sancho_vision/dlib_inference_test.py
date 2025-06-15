#!/usr/bin/env python3
"""Nodo ROS2 para calcular métricas de inferencia con Dlib HOG Face Detector.
Se suscribe a un topic de imágenes, aplica Dlib y publica el tiempo medio de inferencia.
"""
import time

import cv2
import dlib
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image


class DlibInferenceNode(Node):
    """
    A ROS2 node for face detection using Dlib's CNN face detector.

    This node subscribes to an image topic, performs face detection using Dlib's
    CNN-based face detector, and publishes an annotated image with bounding boxes
    around detected faces. It also logs performance metrics for inference time.

    Parameters:
        inference.upsample_times (int): Number of times to upsample the image during detection.
                                        Higher values can detect smaller faces but increase processing time.
                                        Default: 1
        inference.input_topic (str): Topic name for receiving input images.
                                     Default: "/dlib/relay_image"
        inference.output_topic (str): Topic name for publishing annotated images.
                                      Default: "/dlib/detections"

    Subscribes:
        Image: Receives camera images for processing on the input topic.

    Publishes:
        Image: Outputs annotated images with face detections on the output topic.

    Dependencies:
        - dlib
        - cv_bridge
        - OpenCV
        - ROS2 Image message type
    """
    def __init__(self):
        super().__init__("dlib_inference_node")
        self.bridge = CvBridge()

        # Para almacenar los tiempos de inferencia (en segundos)
        self.times = []
        self.first = True  # Para descartar la primera inferencia

        # Parámetros
        self.declare_parameter("inference.upsample_times", 1)
        in_topic = self.declare_parameter(
            "inference.input_topic", "/dlib/relay_image"
        ).value
        out_topic = self.declare_parameter(
            "inference.output_topic", "/dlib/detections"
        ).value

        # Subscripción y publicación
        self.sub = self.create_subscription(
            Image, in_topic, self.image_callback, rclpy.qos.QoSProfile(depth=1)
        )
        self.pub = self.create_publisher(Image, out_topic, 1)

        # Inicializar detector
        up = int(self.get_parameter("inference.upsample_times").value)
        self.detector = dlib.cnn_face_detection_model_v1("/home/ubuntu/sancho_ws/src/sancho_vision/models/mmod_human_face_detector.dat")
        self.upsample_times = up
        self.get_logger().info(f"Dlib HOG detector inicializado (upsample_times={up})")

    def image_callback(self, msg: Image):
        # Convertir ROS Image a OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Medir tiempo de inferencia
        start = time.perf_counter()
        dets = self.detector(img, self.upsample_times)
        inf_time = time.perf_counter() - start

        # Ignorar primera inferencia (pseudoinicialización)
        if self.first:
            self.first = False
            self.times.clear()
            inf_time = 0.0

        # Acumular y calcular media
        self.times.append(inf_time)
        mean_time = sum(self.times) / len(self.times)

        # Log de métricas en milisegundos
        self.get_logger().info(
            f"Inference: {inf_time*1000:.2f} ms — Media: {mean_time*1000:.2f} ms"
        )

        # Dibujar detecciones
        vis = img.copy()
        for rect in dets:
            x1, y1, x2, y2 = rect.left(), rect.top(), rect.right(), rect.bottom()
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Publicar imagen con detecciones
        try:
            out_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge pub error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DlibInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
