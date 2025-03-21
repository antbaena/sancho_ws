import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from human_face_recognition_msgs.srv import Detection
from .detectors.detector_dlib_cnn import get_faces
from .hri_bridge import HRIBridge


class HumanFaceDetector(Node):

    def __init__(self):
        """Initializes the detector node. Creates publisher for detected faces."""

        super().__init__("human_face_detector")

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

        frame = self.br.imgmsg_to_cv2(request.frame, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_equalized = cv2.equalizeHist(gray)

        frame_ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        frame_ycrcb[:, :, 0] = gray_equalized

        frame_equalized = cv2.cvtColor(frame_ycrcb, cv2.COLOR_YCrCb2BGR)

        positions, scores, _ = get_faces(frame_equalized)
        positions_msg, scores_msg = self.br.detector_to_msg(positions, scores)
        self.get_logger().info("Faces detected: " + str(len(positions_msg)))

        response.positions = positions_msg
        response.scores = scores_msg
        i = 0
        for x, y, w, h in positions:
            cv2.rectangle(frame_equalized, (x, y), (x + w, y + h), (255, 0, 0), 2)
            # text = f'({x}, {y})'
            # cv2.putText(frame_equalized, str(idx[i]), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            # i=i+1

        detection_imgmsg = self.br.cv2_to_imgmsg(frame_equalized, "bgr8")
        self.detection_publisher.publish(detection_imgmsg)

        return response


def main(args=None):
    rclpy.init(args=args)

    human_face_detector = HumanFaceDetector()

    rclpy.spin(human_face_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
