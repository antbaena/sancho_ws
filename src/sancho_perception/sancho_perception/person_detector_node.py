#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sancho_msgs.msg import DetectedObjects, DetectedObject, BoundingBox2D

from cv_bridge import CvBridge
from ultralytics import YOLO

class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector_node')
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.detections_pub = self.create_publisher(
            DetectedObjects, '/detected_objects', 10)

        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')  # ejemplo
        self.get_logger().info("Person Detector Node started.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = cv_image.shape

        results = self.yolo_model.predict(cv_image, conf=0.5)
        detection_msg = DetectedObjects()
        detection_msg.header = msg.header  # mantenemos el stamp y frame_id

        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())  # clase
            score = float(box.conf[0].item())
            # Solo personas en COCO
            if cls_id == 0:
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                bb = BoundingBox2D()
                bb.x_min = max(0.0, x1)
                bb.y_min = max(0.0, y1)
                bb.x_max = min(x2, float(width))
                bb.y_max = min(y2, float(height))

                obj = DetectedObject()
                obj.class_id = cls_id
                obj.score = score
                obj.track_id = -1   # sin tracking
                obj.bbox = bb

                detection_msg.objects.append(obj)

        self.detections_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
