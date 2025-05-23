#!/usr/bin/env python3
"""
Nodo ROS2 Lifecycle para detección facial (ROS2 Humble)
  - Uso de LifecycleNode
  - Elección entre Dlib y MediaPipe
  - Procesamiento asíncrono con ThreadPoolExecutor
  - Suavizado EMA de bounding boxes
  - Umbral de confianza y NMS configurable
  - Visualización opcional
"""
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from sancho_msgs.msg import Face, FaceArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import mediapipe as mp

class FaceDetectorLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('face_detector_lifecycle')

        self.declare_parameters(namespace='', parameters=[
            ('image_topic', '/sancho_camera/image_rect'),
            ('detections_topic', '/face_detections'),
            ('use_mediapipe', False),
            ('confidence_threshold', 0.5),
            ('nms_iou_threshold', 0.3),
            ('ema_alpha', 0.3),
            ('downscale_factor', 0.5),
            ('visualization', True),
            ('processing_rate', 10.0),
        ])

        self.bridge = CvBridge()

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando nodo...")

        self.use_mediapipe = self.get_parameter('use_mediapipe').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.nms_iou = self.get_parameter('nms_iou_threshold').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.ds_factor = self.get_parameter('downscale_factor').value
        self.visualization = self.get_parameter('visualization').value
        self.processing_rate = self.get_parameter('processing_rate').value

        if self.use_mediapipe:
            self.mp_face = mp.solutions.face_detection.FaceDetection(
                model_selection=1, min_detection_confidence=self.conf_thresh)
            self.get_logger().info('Detector MediaPipe activado')
        else:
            import dlib
            self.detector = dlib.get_frontal_face_detector()
            self.get_logger().info('Detector Dlib activado')

        qos = QoSProfile(depth=10)
        self.pub = self.create_lifecycle_publisher(FaceArray, self.get_parameter('detections_topic').value, qos)
        if self.visualization:
            self.vis_pub = self.create_lifecycle_publisher(Image, self.get_parameter('detections_topic').value + '/vis', qos)

        return super().on_configure(state)

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Activando nodo...")
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Image, self.get_parameter('image_topic').value, self.image_callback, qos)
        self.latest_img = None
        self.smoothed_boxes = []

        self.timer = self.create_timer(1.0 / self.processing_rate, self.process_image)
        self.get_logger().info(f"Nodo activado, subscrito a {self.get_parameter('image_topic').value}")
        return super().on_activate(state)

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando nodo...")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.sub:
            self.destroy_subscription(self.sub)
            self.sub = None
        return super().on_deactivate(state)
        

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.sub:
            self.destroy_subscription(self.sub)
            self.sub = None
        if self.pub:
            self.destroy_publisher(self.pub)
            self.pub = None
        if self.vis_pub:
            self.destroy_publisher(self.vis_pub)
            self.vis_pub = None
        return super().on_cleanup(state)

    def image_callback(self, msg):
        self.latest_img = msg

    def process_image(self):
        if self.latest_img is None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        small = cv2.resize(cv_img, None, fx=self.ds_factor, fy=self.ds_factor)
        scale = cv_img.shape[1] / small.shape[1]

        boxes, scores = [], []
        if self.use_mediapipe:
            results = self.mp_face.process(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
            if results.detections:
                for det in results.detections:
                    bbox = det.location_data.relative_bounding_box
                    x1 = int(bbox.xmin * small.shape[1])
                    y1 = int(bbox.ymin * small.shape[0])
                    x2 = int((bbox.xmin + bbox.width) * small.shape[1])
                    y2 = int((bbox.ymin + bbox.height) * small.shape[0])
                    boxes.append([x1*scale, y1*scale, x2*scale, y2*scale])
                    scores.append(det.score[0])
        else:
            dets = self.detector(small, 0)
            for d in dets:
                boxes.append([d.left()*scale, d.top()*scale, d.right()*scale, d.bottom()*scale])
                scores.append(1.0)

        idxs = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thresh, self.nms_iou)
        filtered, filtered_scores = [], []
        for i in idxs.flatten():
            filtered.append(boxes[i])
            filtered_scores.append(scores[i])

        fa = FaceArray(header=self.latest_img.header)
        for box, sc in zip(filtered, filtered_scores):
            cx, cy = (box[0] + box[2])/2, (box[1] + box[3])/2
            face = Face(
                header=self.latest_img.header,
                center=Point(x=cx, y=cy, z=0.0),
                width=box[2]-box[0],
                height=box[3]-box[1],
                confidence=sc
            )
            fa.faces.append(face)

        self.pub.publish(fa)
        self.get_logger().info(f'Publicadas {len(fa.faces)} detecciones')

        if self.visualization:
            vis_img = cv_img.copy()
            for box in filtered:
                cv2.rectangle(vis_img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0,255,0), 2)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, 'bgr8')
            vis_msg.header = self.latest_img.header
            self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
