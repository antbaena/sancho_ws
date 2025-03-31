#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from sancho_msgs.msg import PersonsPoses, PersonPose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters

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

        # Publicadores para detecciones 3D y múltiples poses
        self.keypoints3d_pub = self.create_publisher(PersonsPoses, '/human_pose/keypoints3d', 10)
        self.persons_poses_pub = self.create_publisher(PoseArray, '/human_pose/persons_poses', 10)

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

        poses_list = []  # Lista de poses válidas para todas las detecciones

        for person in detections_msg.persons:
            new_person = PersonPose()
            new_person.header = person.header
            new_person.id = person.id
            new_person.keypoints = [0.0] * 34  # Dejamos 2D en cero
            new_person.scores = person.scores

            keypoints_3d = []
            depths = []
            valid_points = []
            # Convertir la lista aplanada en lista de tuplas (x,y)
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
                            valid_points.append(pt3d)
                    else:
                        keypoints_3d.extend([0.0, 0.0, 0.0])
                else:
                    keypoints_3d.extend([0.0, 0.0, 0.0])
            new_person.keypoints3d = keypoints_3d
            new_person.avg_depth = float(np.mean(depths)) if depths else 0.0

            persons_3d_msg.persons.append(new_person)

            # Si existen puntos válidos, calcular la pose promedio para este detection
            if valid_points:
                avg_point = np.mean(valid_points, axis=0)
                pose = Pose()
                pose.position.x = float(avg_point[0])
                pose.position.y = float(avg_point[1])
                pose.position.z = float(avg_point[2])
                # Orientación fija (se puede mejorar el cálculo de la orientación real)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                poses_list.append(pose)

        self.keypoints3d_pub.publish(persons_3d_msg)
        self.get_logger().info("Publicadas detecciones 3D.")

        if poses_list:
            poses_array = PoseArray()
            poses_array.header = detections_msg.header
            poses_array.poses = poses_list
            self.persons_poses_pub.publish(poses_array)
            self.get_logger().info(f"Publicadas {len(poses_list)} poses válidas en persons_poses.")
        else:
            self.get_logger().info("No se pudo calcular la pose 3D de ninguna persona.")

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
