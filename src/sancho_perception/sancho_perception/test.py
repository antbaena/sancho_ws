#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# Se mantienen los custom messages para keypoints2d y keypoints3d
from sancho_msgs.msg import PersonsPoses, PersonPose

import cv2 as cv
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
from cv_bridge import CvBridge, CvBridgeError
import copy
import message_filters

class MoveNetDetectorNode(Node):
    def __init__(self):
        super().__init__('movenet_detector')
        self.get_logger().info("Iniciando nodo MoveNet Detector...")

        # Parámetros configurables
        self.declare_parameter('mirror', False)
        self.declare_parameter('input_size', 256)
        self.declare_parameter('keypoint_score_threshold', 0.4)
        self.declare_parameter('min_keypoints', 9)  # mínimo de keypoints válidos para considerar la detección
        self.declare_parameter('model_url', "https://tfhub.dev/google/movenet/multipose/lightning/1")
        self.declare_parameter('depth_window_size', 3)  # Tamaño de ventana para promediar la profundidad

        self.mirror = self.get_parameter('mirror').value
        self.input_size = self.get_parameter('input_size').value
        self.keypoint_score_threshold = self.get_parameter('keypoint_score_threshold').value
        self.min_keypoints = self.get_parameter('min_keypoints').value
        self.depth_window_size = self.get_parameter('depth_window_size').value

        # Inicializar cv_bridge
        self.bridge = CvBridge()

        # Suscriptores
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        # Sincronizar color y profundidad
        ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],
                                                         queue_size=10, slop=0.1)
        ts.registerCallback(self.image_callback)

        # Publicadores:
        # Publicador para la pose de la persona (PoseStamped estándar)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, '/human_pose/person_pose', 10)
        # Publicadores para los keypoints (custom)
        self.keypoints2d_pub = self.create_publisher(PersonsPoses, '/human_pose/keypoints2d', 10)
        self.keypoints3d_pub = self.create_publisher(PersonsPoses, '/human_pose/keypoints3d', 10)
        # Publicadores para imágenes anotadas
        self.annotated_color_pub = self.create_publisher(Image, '/human_pose/annotated_color', 10)
        self.annotated_depth_pub = self.create_publisher(Image, '/human_pose/annotated_depth', 10)

        # Cargar el modelo MoveNet (multipose lightning)
        self.get_logger().info("Cargando modelo MoveNet...")
        try:
            model_url = self.get_parameter('model_url').value
            self.model = hub.load(model_url).signatures['serving_default']
            self.get_logger().info("Modelo cargado exitosamente.")
        except Exception as e:
            self.get_logger().error(f"Error al cargar el modelo: {e}")
            raise e

        # Almacenará el último mensaje de camera_info recibido
        self.camera_info = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def get_depth_value(self, x, y, depth_image, window_size=3):
        """
        Calcula la profundidad robusta promediando en una ventana centrada en (x,y).
        Se ignoran los valores cero.
        """
        half_w = window_size // 2
        h, w = depth_image.shape
        x_min = max(x - half_w, 0)
        x_max = min(x + half_w + 1, w)
        y_min = max(y - half_w, 0)
        y_max = min(y + half_w + 1, h)
        patch = depth_image[y_min:y_max, x_min:x_max]
        # Seleccionar solo valores mayores a cero
        valid = patch[patch > 0]
        if valid.size == 0:
            return 0.0
        return float(np.mean(valid))

    def run_inference(self, image):
        image_height, image_width = image.shape[:2]
        try:
            input_image = cv.resize(image, (self.input_size, self.input_size))
            input_image = cv.cvtColor(input_image, cv.COLOR_BGR2RGB)
            input_image = input_image.reshape(-1, self.input_size, self.input_size, 3)
            input_image = tf.cast(input_image, dtype=tf.int32)
        except Exception as e:
            self.get_logger().error(f"Error en preprocesamiento: {e}")
            return [], []
        try:
            outputs = self.model(input_image)
            keypoints_with_scores = outputs['output_0'].numpy().squeeze()
        except Exception as e:
            self.get_logger().error(f"Error durante la inferencia: {e}")
            return [], []

        keypoints_list = []
        scores_list = []
        for kp_with_score in keypoints_with_scores:
            keypoints = []
            scores = []
            for i in range(17):  # Se procesan 17 keypoints
                try:
                    x = int(image_width * kp_with_score[i * 3 + 1])
                    y = int(image_height * kp_with_score[i * 3 + 0])
                    score = kp_with_score[i * 3 + 2]
                    keypoints.append((x, y))
                    scores.append(score)
                except Exception as e:
                    self.get_logger().error(f"Error procesando keypoint {i}: {e}")
                    keypoints.append((0, 0))
                    scores.append(0.0)
            keypoints_list.append(keypoints)
            scores_list.append(scores)
        return keypoints_list, scores_list

    def process_detections(self, keypoints_list, scores_list):
        """
        Filtra las detecciones descartando aquellas con menos de 'min_keypoints' válidos.
        """
        valid_detections = []
        for keypoints, scores in zip(keypoints_list, scores_list):
            count_valid = sum([1 for score in scores if score > self.keypoint_score_threshold])
            if count_valid >= self.min_keypoints:
                valid_detections.append((keypoints, scores))
            else:
                self.get_logger().debug(f"Detección descartada: {count_valid} keypoints válidos (< {self.min_keypoints}).")
        return valid_detections

    def convert_2d_to_3d(self, u, v, depth_value):
        """
        Convierte un punto 2D (u,v) a 3D usando el modelo de cámara pinhole y la información de la cámara.
        """
        if self.camera_info is None:
            return (0.0, 0.0, 0.0)
        K = self.camera_info.k  # Matriz de cámara (lista de 9 elementos)
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]
        try:
            X = (u - cx) * depth_value / fx
            Y = (v - cy) * depth_value / fy
            Z = depth_value
            return (X, Y, Z)
        except Exception as e:
            self.get_logger().error(f"Error en conversión 2D->3D: {e}")
            return (0.0, 0.0, 0.0)

    def annotate_depth(self, depth_image):
        """Genera una imagen pseudo-coloreada a partir de la imagen de profundidad."""
        try:
            depth_normalized = cv.normalize(depth_image, None, 0, 255, cv.NORM_MINMAX)
            depth_normalized = np.uint8(depth_normalized)
            depth_colored = cv.applyColorMap(depth_normalized, cv.COLORMAP_JET)
            return depth_colored
        except Exception as e:
            self.get_logger().error(f"Error al anotar imagen de profundidad: {e}")
            return depth_image

    def draw_skeleton(self, image, keypoints, scores):
        """Dibuja el esqueleto sobre la imagen usando las conexiones definidas."""
        connections = [
            (0, 1), (0, 2),
            (1, 3), (2, 4),
            (0, 5), (0, 6),
            (5, 6),
            (5, 7), (7, 9),
            (6, 8), (8, 10),
            (5, 11), (6, 12),
            (11, 12),
            (11, 13), (13, 15),
            (12, 14), (14, 16)
        ]
        annotated = copy.deepcopy(image)
        # Dibujar líneas del esqueleto
        for i, j in connections:
            if scores[i] > self.keypoint_score_threshold and scores[j] > self.keypoint_score_threshold:
                pt1 = keypoints[i]
                pt2 = keypoints[j]
                cv.line(annotated, pt1, pt2, (0, 0, 0), 2)
                cv.line(annotated, pt1, pt2, (255, 255, 255), 4)
        # Dibujar puntos
        for pt, score in zip(keypoints, scores):
            if score > self.keypoint_score_threshold:
                cv.circle(annotated, pt, 6, (0, 0, 0), -1)
                cv.circle(annotated, pt, 3, (255, 255, 255), -1)
        return annotated

    def image_callback(self, color_msg, depth_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Error en conversión de imagen: {e}")
            return

        if self.mirror:
            color_image = cv.flip(color_image, 1)

        keypoints_list, scores_list = self.run_inference(color_image)
        if not keypoints_list:
            self.get_logger().warning("No se detectaron keypoints en la imagen.")
            return

        valid_detections = self.process_detections(keypoints_list, scores_list)
        if not valid_detections:
            self.get_logger().info("No se detectaron personas válidas.")
        else:
            self.get_logger().info(f"Personas detectadas: {len(valid_detections)}")

        # Generar imagen anotada en color y profundidad
        annotated_color = color_image.copy()
        for keypoints, scores in valid_detections:
            annotated_color = self.draw_skeleton(annotated_color, keypoints, scores)
        annotated_depth = self.annotate_depth(depth_image)

        # Preparar mensajes custom para keypoints 2D y 3D
        persons_2d = PersonsPoses()
        persons_3d = PersonsPoses()
        persons_2d.header = color_msg.header
        persons_3d.header = color_msg.header

        # Variables para seleccionar la detección “mejor” y generar el PoseStamped
        best_valid_count = -1
        best_avg_position = None

        person_id = 0
        for keypoints, scores in valid_detections:
            person_msg = PersonPose()
            person_msg.header = color_msg.header
            person_msg.id = person_id

            # Rellenar keypoints 2D (34 valores: [x0, y0, x1, y1, ..., x16, y16])
            keypoints2d = []
            for (x, y) in keypoints:
                keypoints2d.extend([float(x), float(y)])
            person_msg.keypoints = keypoints2d

            # Rellenar scores (17 valores)
            person_msg.scores = [float(s) for s in scores]

            # Calcular keypoints 3D (51 valores: [x0,y0,z0, ...]) y promedio de profundidad
            depths = []
            keypoints3d = []
            valid_points = []
            for (x, y), score in zip(keypoints, scores):
                if score > self.keypoint_score_threshold:
                    if y < depth_image.shape[0] and x < depth_image.shape[1]:
                        # Obtener valor de profundidad promediado en una ventana para mayor robustez
                        depth_val = self.get_depth_value(x, y, depth_image, self.depth_window_size)
                        if depth_val == 0:
                            keypoints3d.extend([0.0, 0.0, 0.0])
                        else:
                            pt3d = self.convert_2d_to_3d(x, y, depth_val)
                            keypoints3d.extend([float(pt3d[0]), float(pt3d[1]), float(pt3d[2])])
                            depths.append(depth_val)
                            valid_points.append(pt3d)
                    else:
                        keypoints3d.extend([0.0, 0.0, 0.0])
                else:
                    keypoints3d.extend([0.0, 0.0, 0.0])
            person_msg.avg_depth = float(np.mean(depths)) if depths else 0.0
            person_msg.keypoints3d = keypoints3d

            # Publicar mensaje para keypoints 2D (keypoints3d en cero)
            person_2d = PersonPose()
            person_2d.header = person_msg.header
            person_2d.id = person_msg.id
            person_2d.keypoints = person_msg.keypoints
            person_2d.scores = person_msg.scores
            person_2d.avg_depth = person_msg.avg_depth
            person_2d.keypoints3d = [0.0] * 51
            persons_2d.persons.append(person_2d)

            # Publicar mensaje para keypoints 3D (keypoints 2D en cero)
            person_3d = PersonPose()
            person_3d.header = person_msg.header
            person_3d.id = person_msg.id
            person_3d.keypoints = [0.0] * 34
            person_3d.scores = person_msg.scores
            person_3d.avg_depth = person_msg.avg_depth
            person_3d.keypoints3d = person_msg.keypoints3d
            persons_3d.persons.append(person_3d)

            # Calcular la posición 3D promedio de esta detección a partir de los puntos válidos
            if valid_points:
                avg_point = np.mean(valid_points, axis=0)
                if len(valid_points) > best_valid_count:
                    best_valid_count = len(valid_points)
                    best_avg_position = avg_point
            person_id += 1

        # Publicar mensajes custom para keypoints 2D y 3D
        self.keypoints2d_pub.publish(persons_2d)
        self.keypoints3d_pub.publish(persons_3d)

        # Publicar imágenes anotadas
        try:
            annotated_color_msg = self.bridge.cv2_to_imgmsg(annotated_color, encoding="bgr8")
            self.annotated_color_pub.publish(annotated_color_msg)
            annotated_depth_msg = self.bridge.cv2_to_imgmsg(annotated_depth, encoding="bgr8")
            self.annotated_depth_pub.publish(annotated_depth_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error publicando imágenes anotadas: {e}")

        # Publicar la pose de la persona (PoseStamped) para la detección “mejor”
        if best_avg_position is not None:
            pose_msg = PoseStamped()
            pose_msg.header = color_msg.header
            pose_msg.pose.position = Point(
                x=float(best_avg_position[0]),
                y=float(best_avg_position[1]),
                z=float(best_avg_position[2])
            )
            # Orientación fija (identidad); se puede mejorar calculando la orientación real
            pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.pose_stamped_pub.publish(pose_msg)
        else:
            self.get_logger().info("No se pudo calcular la pose 3D de ninguna persona.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveNetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo...")
    except Exception as e:
        node.get_logger().error(f"Error inesperado: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
