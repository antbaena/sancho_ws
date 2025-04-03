#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sancho_msgs.msg import PersonsPoses, PersonPose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from people_msgs.msg import People, Person
from geometry_msgs.msg import Vector3

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .movenet_utils import get_depth_value, convert_2d_to_3d

class MoveNetPostprocessingNode(Node):
    def __init__(self):
        super().__init__('movenet_postprocessing')
        self.get_logger().info("Iniciando nodo de postprocesamiento MoveNet...")

        # Parámetros configurables
        self.declare_parameter('depth_window_size', 3)
        self.declare_parameter('keypoint_score_threshold', 0.4)
        self.depth_window_size = self.get_parameter('depth_window_size').value
        self.keypoint_score_threshold = self.get_parameter('keypoint_score_threshold').value

        self.bridge = CvBridge()

        # Suscripción sincronizada a detecciones "raw" y a la imagen de profundidad
        self.detections_sub = message_filters.Subscriber(self, PersonsPoses, '/movenet/raw_detections')
        self.depth_sub = message_filters.Subscriber(self, Image, 'astra_camera/camera/depth/image_raw')
        ts = message_filters.ApproximateTimeSynchronizer([self.detections_sub, self.depth_sub],
                                                          queue_size=10, slop=0.1)
        ts.registerCallback(self.sync_callback)

        # Suscriptor a camera_info para la conversión 2D→3D
        self.create_subscription(CameraInfo, 'astra_camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.camera_info = None

        # Publicadores para detecciones 3D y esqueletos en MarkerArray
        self.keypoints3d_pub = self.create_publisher(PersonsPoses, '/human_pose/keypoints3d', 10)
        self.skeleton_markers_pub = self.create_publisher(MarkerArray, '/human_pose/skeleton_markers', 10)
        self.people_pub = self.create_publisher(People, '/people', 10)


    def camera_info_callback(self, msg):
        self.camera_info = msg

    def sync_callback(self, detections_msg, depth_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen de profundidad: {e}")
            return

        persons_3d_msg = PersonsPoses()
        persons_3d_msg.header = detections_msg.header

        skeleton_marker_array = MarkerArray()

        for person in detections_msg.persons:
            new_person = PersonPose()
            new_person.header = person.header
            new_person.id = person.id
            new_person.keypoints = [0.0] * 34  # Dejamos 2D en cero
            new_person.scores = person.scores

            keypoints_3d = []
            depths = []
            # Convertir la lista aplanada en lista de tuplas (x, y)
            keypoints = [(int(person.keypoints[i]), int(person.keypoints[i+1])) 
                         for i in range(0, len(person.keypoints), 2)]
            for (x, y), score in zip(keypoints, person.scores):
                if score > self.keypoint_score_threshold:
                    if y < depth_image.shape[0] and x < depth_image.shape[1]:
                        depth_val = get_depth_value(x, y, depth_image, self.depth_window_size)
                        if depth_val == 0:
                            keypoints_3d.extend([0.0, 0.0, 0.0])
                        else:
                            pt3d = convert_2d_to_3d(x, y, depth_val, self.camera_info)
                            keypoints_3d.extend([float(pt3d[0]), float(pt3d[1]), float(pt3d[2])])
                            depths.append(depth_val)
                    else:
                        keypoints_3d.extend([0.0, 0.0, 0.0])
                else:
                    keypoints_3d.extend([0.0, 0.0, 0.0])
            new_person.keypoints3d = keypoints_3d
            new_person.avg_depth = float(np.mean(depths)) if depths else 0.0

            persons_3d_msg.persons.append(new_person)

            # Crear marcador para el esqueleto usando un Marker de tipo LINE_LIST
            if len(keypoints_3d) == 51:
                # Convertir la lista de 51 valores en 17 tuplas (x,y,z)
                points_3d = [(keypoints_3d[i], keypoints_3d[i+1], keypoints_3d[i+2]) for i in range(0, 51, 3)]
                # Definir conexiones del esqueleto según MoveNet
                skeleton_connections = [
                    (0, 1),
                    (0, 2),
                    (1, 3),
                    (2, 4),
                    (0, 5),
                    (0, 6),
                    (5, 7),
                    (7, 9),
                    (6, 8),
                    (8, 10),
                    (5, 11),
                    (6, 12),
                    (11, 13),
                    (13, 15),
                    (12, 14),
                    (14, 16)
                ]
                marker = Marker()
                marker.header = detections_msg.header
                marker.ns = "skeleton"
                marker.id = person.id
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # Ancho de la línea
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime.sec = 1
                # Agregar segmentos para cada conexión válida (omitiendo puntos no detectados)
                for idx1, idx2 in skeleton_connections:
                    p1 = points_3d[idx1]
                    p2 = points_3d[idx2]
                    if not (p1 == (0.0, 0.0, 0.0) or p2 == (0.0, 0.0, 0.0)):
                        marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
                        marker.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))

                skeleton_marker_array.markers.append(marker)



        self.keypoints3d_pub.publish(persons_3d_msg)
        
        people_msg = People()
        people_msg.header = detections_msg.header

        for person in persons_3d_msg.persons:
            if person.avg_depth > 0.0:  # Solo personas válidas
                p = Person()
                p.name = f"person_{person.id}"
                p.position = Point(x=float(person.keypoints3d[0]),
                                  y=float(person.keypoints3d[1]),
                                  z=float(person.keypoints3d[2]))
                p.velocity = Point(x=0.0, y=0.0, z=0.0)  # Opcional: podrías estimar velocidad
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
