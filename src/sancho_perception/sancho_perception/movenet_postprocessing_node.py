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

        self.expected_kpts = 17
        self.skel_conns  = [
                    (0, 1), (0, 2), (1, 3), (2, 4),
                    (0, 5), (0, 6), (5, 7), (7, 9),
                    (6, 8), (8, 10), (5, 11), (6, 12),
                    (11, 13), (13, 15), (12, 14), (14, 16)
                ]
        self.point_marker_template = Marker(
            type=Marker.SPHERE_LIST,
            scale=Vector3(0.05,0.05,0.05),
            color=ColorRGBA(1,0,0,1),
            lifetime=Duration(sec=1)
        )


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
        if self.camera_info is None:
            self.get_logger().warn("camera_info aún no disponible, omitiendo procesamiento de este ciclo.")
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen de profundidad: {e}")
            return

        persons_3d_msg = self._procesar_detecciones_3d(detections_msg, depth_image)
        self.keypoints3d_pub.publish(persons_3d_msg)

        people_msg = self._generar_people_msg(detections_msg.header, persons_3d_msg)
        if people_msg.people:
            self.people_pub.publish(people_msg)

        skeleton_marker_array = self._generar_visualizaciones(detections_msg, persons_3d_msg)
        self.skeleton_markers_pub.publish(skeleton_marker_array)

        self.get_logger().info("Publicadas detecciones 3D y esqueletos.")


    def _procesar_detecciones_3d(self, detections_msg, depth_image):
        persons_3d_msg = PersonsPoses()
        persons_3d_msg.header = detections_msg.header

        for person in detections_msg.persons:
            new_person = PersonPose()
            new_person.header = person.header
            new_person.id = person.id
            new_person.scores = person.scores

            keypoints_3d = []
            depths = []
            keypoints = [(int(person.keypoints[i]), int(person.keypoints[i+1])) for i in range(0, len(person.keypoints), 2)]

            for (x, y), score in zip(keypoints, person.scores):
                if score > self.keypoint_score_threshold and 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth_val = get_depth_value(x, y, depth_image, self.depth_window_size)
                    if depth_val and depth_val > 0:
                        pt3d = convert_2d_to_3d(x, y, depth_val, self.camera_info)
                        keypoints_3d.extend([float(pt3d[0]), float(pt3d[1]), float(pt3d[2])])
                        depths.append(depth_val)
                        continue
                keypoints_3d.extend([0.0, 0.0, 0.0])  # por defecto si no hay valor

            valid_count = sum(1 for i in range(0, len(keypoints_3d), 3) if tuple(keypoints_3d[i:i+3]) != (0.0, 0.0, 0.0))
            if valid_count < self.min_valid_keypoints:
                continue  # saltamos personas con muy pocos puntos válidos

            new_person.keypoints = person.keypoints
            new_person.keypoints3d = keypoints_3d
            new_person.avg_depth = float(np.mean(depths)) if depths else 0.0
            persons_3d_msg.persons.append(new_person)

        return persons_3d_msg


    def _generar_visualizaciones(self, detections_msg, persons_3d_msg):
        skeleton_marker_array = MarkerArray()

        for person in persons_3d_msg.persons:
            keypoints_3d = person.keypoints3d
            if len(keypoints_3d) != self.expected_kpts * 3:
                self.get_logger().debug(f"Persona {person.id}: número de keypoints 3D inesperado.")
                continue

            points_3d = {
                idx: (x, y, z)
                for idx, (x, y, z) in enumerate(zip(*[iter(keypoints_3d)] * 3))
                if (x, y, z) != (0.0, 0.0, 0.0)
            }

            skeleton_marker_array.markers.append(self._crear_marker_puntos(detections_msg.header, person.id, points_3d))
            skeleton_marker_array.markers.append(self._crear_marker_lineas(detections_msg.header, person.id, points_3d))
            skeleton_marker_array.markers.append(self._crear_marker_texto(detections_msg.header, person.id, points_3d))

        return skeleton_marker_array


    def _crear_marker_puntos(self, header, person_id, points_3d):
        marker = Marker()
        marker.header = header
        marker.ns = f"skeleton_points_{person_id}"
        marker.id = person_id * 2
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        for pt in points_3d.values():
            marker.points.append(Point(x=pt[0], y=pt[1], z=pt[2]))

        return marker


    def _crear_marker_lineas(self, header, person_id, points_3d):
        marker = Marker()
        marker.header = header
        marker.ns = f"skeleton_lines_{person_id}"
        marker.id = person_id * 2 + 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        for idx1, idx2 in self.skel_conns:
            if idx1 in points_3d and idx2 in points_3d:
                p1 = points_3d[idx1]
                p2 = points_3d[idx2]
                marker.points.extend([Point(x=p1[0], y=p1[1], z=p1[2]), Point(x=p2[0], y=p2[1], z=p2[2])])

        return marker


    def _crear_marker_texto(self, header, person_id, points_3d):
        marker = Marker()
        marker.header = header
        marker.ns = "skeleton_text"
        marker.id = 1000 + person_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.2
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.pose.position.x = points_3d[0][0]
        marker.pose.position.y = points_3d[0][1]
        marker.pose.position.z = points_3d[0][2] + 0.3
        marker.text = f"ID{person_id}"
        marker.lifetime.sec = 1

        return marker


    def _generar_people_msg(self, header, persons_3d_msg):
        people_msg = People()
        people_msg.header = header

        for person in persons_3d_msg.persons:
            if person.avg_depth > 1.0:
                p = Person()
                p.name = f"person_{person.id}"
                x,y,z = person.keypoints3d[0:3]
                if x == 0.0 and y == 0.0 and z == 0.0:
                    continue
                p.position = Point(
                    x=float(person.keypoints3d[0]),
                    y=float(person.keypoints3d[1]),
                    z=float(person.keypoints3d[2])
                )
                p.velocity = Point(x=0.0, y=0.0, z=0.0)
                people_msg.people.append(p)

        return people_msg
