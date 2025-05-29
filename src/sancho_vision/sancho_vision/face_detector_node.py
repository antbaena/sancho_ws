#!/usr/bin/env python3
"""
ROS2 Humble FaceDetector mejorado:
  - Elección entre Dlib y MediaPipe
  - Detección asíncrona con ThreadPoolExecutor
  - Suavizado EMA de bounding boxes
  - Umbral de confianza y NMS configurable
  - Parámetros ROS2 para ajustar rendimiento y calidad
  - Publicación de visualización opcional
"""
import os
import math
import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from sancho_msgs.msg import Face, FaceArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

try:
    import mediapipe as mp
    HAS_MP = True
except ImportError:
    HAS_MP = False

from ament_index_python.packages import get_package_share_directory

class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector')

        # Declaración de parámetros
        self.declare_parameter('image_topic', '/sancho_camera/image_rect')
        self.declare_parameter('detections_topic', '/face_detections')
        self.declare_parameter('use_mediapipe', False)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_iou_threshold', 0.3)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('max_workers', 2)
        self.declare_parameter('downscale_factor', 0.5)
        self.declare_parameter('visualization', True)
        self.declare_parameter('processing_rate', 10.0)

        # Obtener parámetros
        image_topic       = self.get_parameter('image_topic').value
        detections_topic  = self.get_parameter('detections_topic').value
        self.use_mediapipe = self.get_parameter('use_mediapipe').value and HAS_MP
        self.conf_thresh   = self.get_parameter('confidence_threshold').value
        self.nms_iou       = self.get_parameter('nms_iou_threshold').value
        self.ema_alpha     = self.get_parameter('ema_alpha').value
        max_workers        = int(self.get_parameter('max_workers').value)
        ds_factor          = self.get_parameter('downscale_factor').value
        self.visualization= self.get_parameter('visualization').value
        rate_hz           = self.get_parameter('processing_rate').value

        # CvBridge
        self.bridge = CvBridge()

        # Inicializar detector
        if self.use_mediapipe:
            self.get_logger().info('Usando MediaPipe Selfie Face Detector')
            self.mp_face = mp.solutions.face_detection.FaceDetection(
                model_selection=1, min_detection_confidence=self.conf_thresh)
        else:
            # Dlib HOG
            import dlib
            self.get_logger().info('Usando Dlib HOG Face Detector')
            self.detector = dlib.get_frontal_face_detector()

        # Subscripción y publicación
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Image, image_topic,
                                           self._on_image, qos)
        self.pub = self.create_publisher(FaceArray, detections_topic, qos)
        if self.visualization:
            self.vis_pub = self.create_publisher(Image,
                                                detections_topic + '/vis', qos)

        # Variables de estado
        self.latest_msg = None
        self.smoothed_boxes = []

        # Timer para procesamiento periódico
        self.create_timer(1.0/rate_hz, self._process)
        self.get_logger().info(f'Procesando a {rate_hz:.1f} Hz')

    def _on_image(self, msg):
        self.latest_msg = msg

    def _process(self):
        msg = self.latest_msg
        self.latest_msg = None
        if msg is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Opcional: reducir resolución para velocidad
        small = cv2.resize(cv_img, None, fx= self.get_parameter('downscale_factor').value,
                           fy= self.get_parameter('downscale_factor').value)
        h0, w0 = cv_img.shape[:2]
        scale = w0 / small.shape[1]

        # Detección: síncrona con MediaPipe o Dlib
        if self.use_mediapipe:
            results = self.mp_face.process(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
            boxes, scores = [], []
            if results.detections:
                for det in results.detections:
                    score = det.score[0]
                    if score < self.conf_thresh:
                        continue
                    bbox = det.location_data.relative_bounding_box
                    x1 = int(bbox.xmin * small.shape[1])
                    y1 = int(bbox.ymin * small.shape[0])
                    x2 = int((bbox.xmin + bbox.width) * small.shape[1])
                    y2 = int((bbox.ymin + bbox.height) * small.shape[0])
                    boxes.append([x1*scale, y1*scale, x2*scale, y2*scale])
                    scores.append(score)
        else:
            dets = self.detector(small, 0)
            boxes, scores = [], []
            for d in dets:
                x1, y1 = d.left(), d.top()
                x2, y2 = d.right(), d.bottom()
                boxes.append([x1*scale, y1*scale, x2*scale, y2*scale])
                scores.append(1.0)

        # NMS: eliminar índices inválidos de NMSBoxes
        idxs = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thresh, self.nms_iou)
        filtered, filtered_scores = [], []
        if idxs is not None and len(idxs) > 0:
            # idxs puede ser lista de int, lista de [int], o np.ndarray
            if isinstance(idxs, np.ndarray):
                flat = idxs.flatten()
            else:
                flat = [i[0] if isinstance(i, (list, tuple, np.ndarray)) else int(i) for i in idxs]
            for i in flat:
                if 0 <= i < len(boxes):
                    filtered.append(boxes[i])
                    filtered_scores.append(scores[i])

        # Suavizado EMA
        # if not self.smoothed_boxes:
        #     self.smoothed_boxes = filtered
        # else:
        #     out = []
        #     for i, box in enumerate(filtered):
        #         if i < len(self.smoothed_boxes):
        #             prev = self.smoothed_boxes[i]
        #             box_s = [self.ema_alpha*b + (1-self.ema_alpha)*p for b, p in zip(box, prev)]
        #         else:
        #             box_s = box
        #         out.append(box_s)
        #     self.smoothed_boxes = out

        # Publicar FaceArray
        fa = FaceArray()
        fa.header = msg.header
        vis_img = cv_img.copy()
        for (x1,y1,x2,y2), sc in zip(filtered, filtered_scores):
            cx = (x1+x2)/2.0
            cy = (y1+y2)/2.0
            w  = x2-x1
            h  = y2-y1
            pt = Point(x=cx, y=cy, z=0.0)
            f = Face(header=msg.header, center=pt,
                     width=float(w), height=float(h), confidence=float(sc))
            fa.faces.append(f)
            if self.visualization:
                cv2.rectangle(vis_img,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
                cv2.putText(vis_img,f'{sc:.2f}',(int(x1),int(y1)-5),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
        self.pub.publish(fa)
        self.get_logger().info(f'Publicadas {len(fa.faces)} caras')

        if self.visualization:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(vis_img, 'bgr8')
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge vis error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
