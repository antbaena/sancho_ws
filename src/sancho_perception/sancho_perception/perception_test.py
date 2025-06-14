#!/usr/bin/env python3
"""
Nodo ROS2 para calcular métricas de inferencia con MoveNet.
Se suscribe a un topic de imágenes, aplica MoveNet y publica el tiempo medio de inferencia.
"""
import time
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from .movenet_utils import load_model, run_inference_on_image, process_detections

class InferenceNode(Node):
    def __init__(self):
        super().__init__('movenet_inference_node')
        self.bridge = CvBridge()

        # Para almacenar los tiempos de inferencia (en segundos)
        self.times = []
        self.first = True  # Para medir el primer tiempo de inferencia
        # parámetros
        self.declare_parameter('inference.model_url', 'https://tfhub.dev/google/movenet/multipose/lightning/1')
        self.declare_parameter('inference.input_size', 256)
        self.declare_parameter('inference.score_threshold', 0.4)
        self.declare_parameter('inference.min_keypoints', 9)

        in_t = '/movenet/relay_image'
        out_t = '/movenet/detections'
        self.sub = self.create_subscription(
            Image, in_t, self.image_callback,
            rclpy.qos.QoSProfile(depth=1)
        )
        self.pub = self.create_publisher(Image, out_t, 1)

        # cargar modelo
        url = self.get_parameter('inference.model_url').value
        size = int(self.get_parameter('inference.input_size').value)
        try:
            self.model = load_model(url)
            self.input_size = size
            self.score_th = float(self.get_parameter('inference.score_threshold').value)
            self.min_kp = int(self.get_parameter('inference.min_keypoints').value)
            self.get_logger().info(f'Modelo cargado: {url}')
        except Exception as e:
            self.get_logger().error(f'Error al cargar MoveNet: {e}')
            raise

    def image_callback(self, msg: Image):
        # convertir mensaje a OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            return

        # medir tiempo de inferencia
        start_time = time.perf_counter()
        kpts, scores = run_inference_on_image(img, self.input_size, self.model)
        inf_time = time.perf_counter() - start_time
        
        # Descartar el primer tiempo de inferencia (puede incluir inicialización)
        if self.first:
            self.first = False
            self.times = []  # Reiniciar para no incluir la primera inferencia
            inf_time = 0.0   # No registrar este tiempo

        # acumular y calcular media
        self.times.append(inf_time)
        mean_time = sum(self.times) / len(self.times)

        # log de métricas en milisegundos
        self.get_logger().info(
            f"Inference: {inf_time*1000:.2f} ms — Media: {mean_time*1000:.2f} ms"
        )

        # procesar detecciones y visualizar
        dets = process_detections(kpts, scores, self.score_th, self.min_kp)
        vis = img.copy()
        for (pts, sc) in dets:
            for idx, (x, y) in enumerate(pts):
                if sc[idx] >= self.score_th:
                    cv2.circle(vis, (int(x), int(y)), 3, (0,255,0), -1)

        # publicar imagen con marcadores
        out_msg = self.bridge.cv2_to_imgmsg(vis, 'bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
