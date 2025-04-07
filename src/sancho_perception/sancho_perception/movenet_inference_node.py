#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sancho_msgs.msg import PersonsPoses, PersonPose
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from .movenet_utils import load_model, run_inference_on_image, process_detections


class MoveNetInferenceNode(LifecycleNode):
    def __init__(self):
        super().__init__('movenet_inference')
        self.get_logger().info("Iniciando nodo de inferencia MoveNet...")
        self.bridge = CvBridge()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando nodo...")
        # Declarar parámetros configurables con valores predeterminados
        self.declare_parameter('mirror', False)
        self.declare_parameter('input_size', 256)
        self.declare_parameter('keypoint_score_threshold', 0.4)
        self.declare_parameter('min_keypoints', 9)
        self.declare_parameter('model_url', "https://tfhub.dev/google/movenet/multipose/lightning/1")
        self.declare_parameter('visualize_markers', True)
        self.declare_parameter('image_topic', '/astra_camera/camera/color/image_raw')
        self.declare_parameter('debug', False)

        # Obtener los parámetros iniciales
        self.mirror = self.get_parameter('mirror').value
        self.input_size = self.get_parameter('input_size').value
        self.keypoint_score_threshold = self.get_parameter('keypoint_score_threshold').value
        self.min_keypoints = self.get_parameter('min_keypoints').value
        self.model_url = self.get_parameter('model_url').value
        self.visualize_markers = self.get_parameter('visualize_markers').value
        self.image_topic = self.get_parameter('image_topic').value
        self.debug = self.get_parameter('debug').value

        # Callback para actualizar parámetros dinámicamente
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Cargar modelo MoveNet
        self.get_logger().info("Cargando modelo MoveNet...")
        try:
            self.model = load_model(self.model_url)
            self.get_logger().info("Modelo cargado exitosamente.")
        except Exception as e:
            self.get_logger().error(f"Error al cargar el modelo: {e}")
            return TransitionCallbackReturn.FAILURE

    
        # Crear suscriptor con QoS adecuado para datos de sensores
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile)

        # Publicador para las detecciones "raw"
        self.detections_pub = self.create_lifecycle_publisher(PersonsPoses, '/movenet/raw_detections', 10)

        # Publicador para la visualización de marcadores, si está habilitado
        if self.visualize_markers:
            self.marker_pub = self.create_lifecycle_publisher(Image, self.image_topic + "/markers", 10)

        return TransitionCallbackReturn.SUCCESS

    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activando nodo...")
        self.detections_pub.activate()
        if self.visualize_markers:
            self.marker_pub.activate()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando nodo...")
        self.detections_pub.deactivate()
        if self.visualize_markers:
            self.marker_pub.deactivate()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando nodo...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Apagando nodo...")
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error("Nodo en estado de error.")
        return TransitionCallbackReturn.SUCCESS

    
    def parameter_update_callback(self, params):
        """
        Callback para manejar cambios dinámicos en los parámetros.
        Permite ajustar la configuración sin reiniciar el nodo.
        """
        for param in params:
            if param.name == 'mirror':
                self.mirror = param.value
                self.get_logger().info(f"Parámetro 'mirror' actualizado: {self.mirror}")
            elif param.name == 'input_size':
                self.input_size = param.value
                self.get_logger().info(f"Parámetro 'input_size' actualizado: {self.input_size}")
            elif param.name == 'keypoint_score_threshold':
                self.keypoint_score_threshold = param.value
                self.get_logger().info(f"Parámetro 'keypoint_score_threshold' actualizado: {self.keypoint_score_threshold}")
            elif param.name == 'min_keypoints':
                self.min_keypoints = param.value
                self.get_logger().info(f"Parámetro 'min_keypoints' actualizado: {self.min_keypoints}")
            elif param.name == 'model_url':
                # Actualizar el modelo en tiempo de ejecución (puede ser costoso)
                self.model_url = param.value
                self.get_logger().info(f"Parámetro 'model_url' actualizado: {self.model_url}")
                try:
                    self.model = load_model(self.model_url)
                    self.get_logger().info("Modelo actualizado exitosamente.")
                except Exception as e:
                    self.get_logger().error(f"Error al actualizar el modelo: {e}")
            elif param.name == 'visualize_markers':
                self.visualize_markers = param.value
                self.get_logger().info(f"Parámetro 'visualize_markers' actualizado: {self.visualize_markers}")
            elif param.name == 'image_topic':
                self.image_topic = param.value
                self.get_logger().info(f"Parámetro 'image_topic' actualizado: {self.image_topic}")
            elif param.name == 'debug':
                self.debug = param.value
                self.get_logger().info(f"Parámetro 'debug' actualizado: {self.debug}")
        return SetParametersResult(successful=True)

    def image_callback(self, color_msg):
        """
        Callback para procesar cada frame de imagen, ejecutar la inferencia con MoveNet,
        y publicar tanto las detecciones como la visualización de marcadores.
        """
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        if self.mirror:
            color_image = cv.flip(color_image, 1)

        # Ejecutar la inferencia MoveNet
        try:
            keypoints_list, scores_list = run_inference_on_image(color_image, self.input_size, self.model)
        except Exception as e:
            self.get_logger().error(f"Error durante la inferencia: {e}")
            return

        # Procesar las detecciones obtenidas
        valid_detections = process_detections(keypoints_list, scores_list,
                                              self.keypoint_score_threshold, self.min_keypoints)

        detections_msg = PersonsPoses()
        detections_msg.header = color_msg.header

        # Si se habilita la visualización, se utiliza una copia de la imagen para dibujar
        if self.visualize_markers:
            vis_image = color_image.copy()

        for person_id, (keypoints, scores) in enumerate(valid_detections):
            person = PersonPose()
            person.header = color_msg.header
            person.id = person_id
            # Aplanar la lista de keypoints (cada punto se convierte en dos valores: x, y)
            person.keypoints = [float(coord) for point in keypoints for coord in point]
            person.scores = [float(s) for s in scores]
            # Valores por defecto para keypoints3d y avg_depth
            person.keypoints3d = [0.0] * 51
            person.avg_depth = 0.0
            detections_msg.persons.append(person)

            # Dibujar los marcadores en la imagen para visualización
            if self.visualize_markers:
                for i, point in enumerate(keypoints):
                    x, y = int(point[0]), int(point[1])
                    if scores[i] >= self.keypoint_score_threshold:
                        cv.circle(vis_image, (x, y), 5, (0, 255, 0), -1)
                        cv.putText(vis_image, str(i), (x + 5, y - 5),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publicar el mensaje de detecciones
        self.detections_pub.publish(detections_msg)
        if self.debug:
            self.get_logger().debug(f"Detecciones publicadas: {len(valid_detections)}")

        # Publicar la imagen con los marcadores, si corresponde
        if self.visualize_markers:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
                vis_msg.header = color_msg.header
                self.marker_pub.publish(vis_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Error al convertir imagen para visualización: {e}")
