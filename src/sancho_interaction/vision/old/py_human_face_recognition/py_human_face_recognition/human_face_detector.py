import cv2
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from human_face_recognition_msgs.srv import Detection
from .hri_bridge import HRIBridge

class HumanFaceDetector(Node):

    def __init__(self):
        """Initializes the detector node. Creates publisher for detected faces."""
        super().__init__("human_face_detector")

        show_metrics_param = self.declare_parameter("show_metrics", False)
        self.show_metrics = show_metrics_param.get_parameter_value().bool_value
        self.get_logger().info("Show Metrics: " + str(self.show_metrics))

        active_cnn_param = self.declare_parameter("active_cnn", False)
        active_cnn = active_cnn_param.get_parameter_value().bool_value
        self.get_logger().info("Active CNN: " + str(active_cnn))

        self.get_faces = None
        
        if active_cnn:
            from .detectors.detector_dlib_cnn import get_faces
            self.get_logger().info("Importing DLIB CNN")
            self.get_faces = get_faces
        else:
            from .detectors.detector_dlib_frontal import get_faces
            self.get_logger().info("Importing DLIB FRONTAL")
            self.get_faces = get_faces

        self.detection_service = self.create_service(
            Detection, "detection", self.detection
        )
        self.detection_publisher = self.create_publisher(
            Image, "camera/color/detection", 10
        )

        self.last_detection_time = 0
        self.br = HRIBridge()

    def detection(self, request, response):
        """Detection service.

        Args:
            request (Detection.srv): The frame.

        Returns:
            response (Detection.srv): Array of 4 elements with positions of the faces and array of scores.
        """

        start_detection = time.time()

        frame = self.br.imgmsg_to_cv2(request.frame, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
  
        image_bilateral = cv2.bilateralFilter(gray, d=5, sigmaColor=50, sigmaSpace=50)
        gray_equalized = cv2.equalizeHist(image_bilateral) 
        
        frame_ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        frame_ycrcb[:, :, 0] = gray_equalized
        frame_equalized = cv2.cvtColor(frame_ycrcb, cv2.COLOR_YCrCb2BGR)

        postProcess_image = frame_equalized
        if self.get_faces:
            positions, scores, _ = self.get_faces(postProcess_image)
            positions_msg, scores_msg = self.br.detector_to_msg(positions, scores)
            self.get_logger().info("Faces detected: " + str(len(positions_msg)))

            response.positions = positions_msg
            response.scores = scores_msg

            for x, y, w, h in positions:
                cv2.rectangle(postProcess_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            detection_imgmsg = self.br.cv2_to_imgmsg(postProcess_image, "bgr8")
            self.detection_publisher.publish(detection_imgmsg)

        detection_time = time.time() - start_detection
        response.detection_time = detection_time
        if self.show_metrics:
            self.get_logger().info("Detection time: " + str(detection_time))

        return response


def main(args=None):
    rclpy.init(args=args)
    human_face_detector = HumanFaceDetector()
    rclpy.spin(human_face_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
