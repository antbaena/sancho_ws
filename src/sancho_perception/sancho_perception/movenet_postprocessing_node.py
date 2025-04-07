#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Image, CameraInfo
from sancho_msgs.msg import PersonsPoses, PersonPose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from people_msgs.msg import People, Person
from geometry_msgs.msg import Vector3
import random
random.seed(42)
# Opcional: semilla para reproducibilidad

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .movenet_utils import get_depth_value, convert_2d_to_3d

class MoveNetPostprocessingNode(LifecycleNode):
    def __init__(self):
        super().__init__('movenet_postprocessing')
        self.get_logger().info("Iniciando nodo de postprocesamiento MoveNet...")

        # Inicializa todo a None, se configurarán en on_configure
        self.bridge = None
        self.detections_sub = None
        self.depth_sub = None
        self.sync = None
        self.camera_info = None

        self.keypoints3d_pub = None
        self.skeleton_markers_pub = None
        self.people_pub = None

        self.declare_parameter('depth_window_size', 3)
        self.declare_parameter('keypoint_score_threshold', 0.4)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configurando MoveNetPostprocessingNode...')

        self.depth_window_size = self.get_parameter('depth_window_size').value
        self.keypoint_score_threshold = self.get_parameter('keypoint_score_threshold').value

        self.bridge = CvBridge()


        # Suscriptor a camera_info para la conversión 2D→3D
        self.create_subscription(CameraInfo, 'astra_camera/camera/color/camera_info', self.camera_info_callback, 10)

        # Publicadores para detecciones 3D y esqueletos en MarkerArray
        self.keypoints3d_pub = self.create_lifecycle_publisher(PersonsPoses, '/human_pose/keypoints3d', 10)
        self.skeleton_markers_pub = self.create_lifecycle_publisher(MarkerArray, '/human_pose/skeleton_markers', 10)
        self.people_pub = self.create_lifecycle_publisher(People, '/people', 10)

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activando MoveNetPostprocessingNode...')

        # Crear los subscribers sincronizados
        self.detections_sub = message_filters.Subscriber(self, PersonsPoses, '/movenet/raw_detections')
        self.depth_sub = message_filters.Subscriber(self, Image, 'astra_camera/camera/depth/image_raw')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.detections_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Desactivando MoveNetPostprocessingNode...')

        # limpiar tus suscripciones (destruir los objetos de message_filters)
        if self.sync:
            self.sync = None
        if self.detections_sub:
            self.detections_sub = None
        if self.depth_sub:
            self.depth_sub = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Limpiando MoveNetPostprocessingNode...')
        # Limpiar recursos
        self.bridge = None
        self.camera_info = None
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('Error en MoveNetPostprocessingNode, limpiando recursos...')
        
        # Aquí podrías limpiar todo lo necesario
        self.bridge = None
        self.camera_info = None
        self.sync = None
        self.detections_sub = None
        self.depth_sub = None
        
        return TransitionCallbackReturn.SUCCESS

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def sync_callback(self, detections_msg, depth_msg):
        # Verificar que camera_info esté disponible
        if self.camera_info is None:
            self.get_logger().warn("camera_info aún no disponible, omitiendo procesamiento de este ciclo.")
            return

        # Convertir imagen de profundidad
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen de profundidad: {e}")
            return

        persons_3d_msg = PersonsPoses()
        persons_3d_msg.header = detections_msg.header
        skeleton_marker_array = MarkerArray()

        # Constante: número esperado de keypoints (por ejemplo, 17)
        EXPECTED_KEYPOINTS = 17

        for person in detections_msg.persons:
            new_person = PersonPose()
            new_person.header = person.header
            new_person.id = person.id
            new_person.keypoints = [0.0] * 34  # Dejamos los keypoints 2D en cero
            new_person.scores = person.scores

            keypoints_3d = []
            depths = []

            # Convertir la lista aplanada en una lista de tuplas (x, y)
            keypoints = [(int(person.keypoints[i]), int(person.keypoints[i+1]))
                        for i in range(0, len(person.keypoints), 2)]
            for (x, y), score in zip(keypoints, person.scores):
                if score > self.keypoint_score_threshold and (0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]):
                    depth_val = get_depth_value(x, y, depth_image, self.depth_window_size)
                    if depth_val == 0:
                        keypoints_3d.extend([0.0, 0.0, 0.0])
                    else:
                        pt3d = convert_2d_to_3d(x, y, depth_val, self.camera_info)
                        keypoints_3d.extend([float(pt3d[0]), float(pt3d[1]), float(pt3d[2])])
                        depths.append(depth_val)
                else:
                    keypoints_3d.extend([0.0, 0.0, 0.0])
            new_person.keypoints3d = keypoints_3d
            new_person.avg_depth = float(np.mean(depths)) if depths else 0.0

            persons_3d_msg.persons.append(new_person)

            # Procesar marcadores solo si se tienen el número esperado de keypoints 3D
            if len(keypoints_3d) == EXPECTED_KEYPOINTS * 3:
                # Convertir la lista plana en una lista de tuplas (x,y,z)
                points_3d = [(keypoints_3d[i], keypoints_3d[i+1], keypoints_3d[i+2])
                            for i in range(0, len(keypoints_3d), 3)]
                # Conexiones del esqueleto (según MoveNet)
                skeleton_connections = [
                    (0, 1), (0, 2), (1, 3), (2, 4),
                    (0, 5), (0, 6), (5, 7), (7, 9),
                    (6, 8), (8, 10), (5, 11), (6, 12),
                    (11, 13), (13, 15), (12, 14), (14, 16)
                ]

                # Marker para los puntos (articulaciones)
                points_marker = Marker()
                points_marker.header = detections_msg.header
                points_marker.ns = f"skeleton_points_{person.id}"
                points_marker.id = person.id * 2  # ID único
                points_marker.type = Marker.SPHERE_LIST
                points_marker.action = Marker.ADD
                points_marker.scale.x = 0.05
                points_marker.scale.y = 0.05
                points_marker.scale.z = 0.05
                points_marker.color.r = 1.0
                points_marker.color.g = 0.0
                points_marker.color.b = 0.0
                points_marker.color.a = 1.0
                points_marker.lifetime.sec = 1
                for point in points_3d:
                    if point != (0.0, 0.0, 0.0):
                        points_marker.points.append(Point(x=point[0], y=point[1], z=point[2]))
                skeleton_marker_array.markers.append(points_marker)

                # Marker para los huesos (líneas)
                lines_marker = Marker()
                lines_marker.header = detections_msg.header
                lines_marker.ns = f"skeleton_lines_{person.id}"
                lines_marker.id = person.id * 2 + 1  # ID único
                lines_marker.type = Marker.LINE_LIST
                lines_marker.action = Marker.ADD
                lines_marker.scale.x = 0.02  # Grosor de la línea
                # Generar color aleatorio para las líneas
                color_r = random.random()
                color_g = random.random()
                color_b = random.random()
                lines_marker.color.r = color_r
                lines_marker.color.g = color_g
                lines_marker.color.b = color_b
                lines_marker.color.a = 1.0
                lines_marker.lifetime.sec = 1
                for idx1, idx2 in skeleton_connections:
                    p1 = points_3d[idx1]
                    p2 = points_3d[idx2]
                    if not (p1 == (0.0, 0.0, 0.0) or p2 == (0.0, 0.0, 0.0)):
                        lines_marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
                        lines_marker.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))
                skeleton_marker_array.markers.append(lines_marker)

                # Marker de texto para mostrar el ID de la persona
                text_marker = Marker()
                text_marker.header = detections_msg.header
                text_marker.ns = "skeleton_text"
                text_marker.id = 1000 + person.id  # Evitar colisión de IDs
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = 0.2  # Tamaño del texto
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.pose.position.x = points_3d[0][0]
                text_marker.pose.position.y = points_3d[0][1]
                text_marker.pose.position.z = points_3d[0][2] + 0.3  # Posicionar el texto por encima de la cabeza
                text_marker.text = f"ID{person.id}"
                text_marker.lifetime.sec = 1
                skeleton_marker_array.markers.append(text_marker)
            else:
                self.get_logger().debug(f"Persona {person.id}: número de keypoints 3D inesperado ({len(keypoints_3d)} valores).")

        # Publicar mensaje de personas 3D
        self.keypoints3d_pub.publish(persons_3d_msg)

        # Construir y publicar mensaje de People
        people_msg = People()
        people_msg.header = detections_msg.header
        for person in persons_3d_msg.persons:
            # Se publica solo si la profundidad promedio es razonable (> 1.0)
            if person.avg_depth > 1.0:
                p = Person()
                p.name = f"person_{person.id}"
                p.position = Point(
                    x=float(person.keypoints3d[0]),
                    y=float(person.keypoints3d[1]),
                    z=float(person.keypoints3d[2])
                )
                p.velocity = Point(x=0.0, y=0.0, z=0.0)  # Se puede estimar la velocidad si es necesario
                people_msg.people.append(p)
        if people_msg.people:
            self.people_pub.publish(people_msg)

        self.skeleton_markers_pub.publish(skeleton_marker_array)
        self.get_logger().info("Publicadas detecciones 3D y esqueletos.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveNetPostprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo de postprocesamiento...")
    except Exception as e:
        node.get_logger().error(f"Error inesperado: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
