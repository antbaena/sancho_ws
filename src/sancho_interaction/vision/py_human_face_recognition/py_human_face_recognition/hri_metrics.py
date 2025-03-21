import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from human_face_recognition_msgs.srv import Detection, Recognition
from .hri_bridge import HRIBridge

class HRIMetricsNode(Node):

    def __init__(self):
        super().__init__("hri_metrics")

        self.last_frame_msg = None
        self.subscription_camera = self.create_subscription(Image, "camera/color/image_raw", self.frame_callback, 1)
  
        self.detection_client = self.create_client(Detection, "detection")
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detection service not available, waiting again...")

        self.recognition_client = self.create_client(Recognition, "recognition")
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Recognition service not available, waiting again...")
        
        self.br = HRIBridge()

    def frame_callback(self, frame_msg):
        self.last_frame_msg = frame_msg

    
class HRIMetrics:

    def __init__(self):       
        self.node = HRIMetricsNode()

    def spin(self):
        while rclpy.ok():
            if self.node.last_frame_msg is not None:
                self.process_frame_metrics(self.node.last_frame_msg)
          
            rclpy.spin_once(self.node)
        
    def process_frame_metrics(self, frame_msg):
        send_detection = time.time()
        positions_msg, scores_msg, detection_time = self.detection_request(frame_msg)  # Detection
        detection_response_time = time.time() - send_detection
        detection_sending_time = detection_response_time - detection_time

        self.node.get_logger().info("Detection response time: " + str(detection_response_time))
        self.node.get_logger().info("Detection processing time: " + str(detection_time))
        self.node.get_logger().info("Time lost while sending and receiving detection data: " + str(detection_sending_time))

        positions, _ = self.node.br.msg_to_detector(positions_msg, scores_msg)

        sum_recognition_response_time, sum_recognition_time = 0, 0
        for i in range(len(positions)):
            send_recognition = time.time()
            _, _, _, _, _, recognition_time = self.recognition_request(frame_msg, positions_msg[i])  
            recognition_response_time = time.time() - send_recognition

            sum_recognition_response_time += recognition_response_time
            sum_recognition_time += recognition_time
        
        sum_recognition_sending_time = sum_recognition_response_time - sum_recognition_time
        if len(positions) > 0:
            self.node.get_logger().info("Sum of recognition response time (" + str(len(positions)) + " faces): " + str(sum_recognition_response_time))
            self.node.get_logger().info("Sum of recognition processing time (" + str(len(positions)) + " faces): " + str(sum_recognition_time))
            self.node.get_logger().info("Sum of time lost while sending and receiving recognition data (" + str(len(positions)) + " faces): " + str(sum_recognition_sending_time))
        
        total_response_time = detection_response_time + sum_recognition_response_time
        total_process_time = detection_time + sum_recognition_time
        total_sending_time = detection_sending_time + sum_recognition_sending_time

        self.node.get_logger().info("Total time: " + str(total_response_time))
        self.node.get_logger().info("Total processing time: " + str(total_process_time))
        self.node.get_logger().info("Total time lost while sending and receiving all data: " + str(total_sending_time))

    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.node.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores, result_detection.detection_time

    def recognition_request(self, frame_msg, position_msg):
        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg

        future_recognition = self.node.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self.node, future_recognition)
        result_recognition = future_recognition.result()

        return (
            result_recognition.face_aligned,
            result_recognition.features,
            result_recognition.classified,
            result_recognition.distance,
            result_recognition.pos,
            result_recognition.recognition_time
        )


def main(args=None):
    rclpy.init(args=args)

    hri_metrics = HRIMetrics()
    hri_metrics.spin()

    rclpy.shutdown()
