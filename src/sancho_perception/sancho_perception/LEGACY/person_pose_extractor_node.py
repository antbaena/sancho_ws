#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Mensajes
from sensor_msgs.msg import Image
from sancho_msgs.msg import DetectedObjects, PersonsPoses, PersonPose

from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
import mediapipe as mp


class PersonPoseNode(Node):
    def __init__(self):
        super().__init__('person_pose_extractor_node')

        # Necesitamos suscribir a 2 temas usando message_filters
        self.image_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.det_sub = Subscriber(self, DetectedObjects, '/detected_objects')

        # Configuramos la sincronización aproximada
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.det_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # Publicador de poses
        self.poses_pub = self.create_publisher(
            PersonsPoses,
            '/persons_poses',
            10
        )

        # Herramientas
        self.bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        # Inicialización de MediaPipe Pose
        self.pose_detector = self.mp_pose.Pose(
            static_image_mode=True,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("Person Pose Node started.")

    def sync_callback(self, image_msg, detected_msg):
        """
        Se llama cuando hay un par sincronizado de (Image, DetectedObjects).
        """
        # Convertir la imagen
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        # Creamos un mensaje de salida
        persons_poses_msg = PersonsPoses()
        persons_poses_msg.header = image_msg.header  # o podrías usar detected_msg.header

        # Para cada objeto detectado
        for obj in detected_msg.objects:
            # Opcional: chequear si class_id = 0 (persona), etc.
            # Recortar
            x_min = int(obj.bbox.x_min)
            y_min = int(obj.bbox.y_min)
            x_max = int(obj.bbox.x_max)
            y_max = int(obj.bbox.y_max)

            # Validar rangos
            x_min = max(0, min(x_min, width))
            y_min = max(0, min(y_min, height))
            x_max = max(0, min(x_max, width))
            y_max = max(0, min(y_max, height))

            if x_max <= x_min or y_max <= y_min:
                # BB inválido
                continue

            person_roi = cv_image[y_min:y_max, x_min:x_max]

            # Procesar con MediaPipe (requiere RGB)
            person_rgb = cv2.cvtColor(person_roi, cv2.COLOR_BGR2RGB)
            results = self.pose_detector.process(person_rgb)

            if results.pose_landmarks:
                # Extraer los keypoints
                keypoints = []
                for lm in results.pose_landmarks.landmark:
                    # lm.x, lm.y en [0..1] con respecto a "person_roi"
                    keypoints.append(lm.x)
                    keypoints.append(lm.y)
                    # Si quisieras lm.z, tendrías que incluirlo (z relativo)

                # Crear mensaje PersonPose
                pose_msg = PersonPose()
                pose_msg.track_id = obj.track_id
                pose_msg.keypoints = keypoints

                # Agregarlo al array
                persons_poses_msg.poses.append(pose_msg)

        # Publicar
        self.poses_pub.publish(persons_poses_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
