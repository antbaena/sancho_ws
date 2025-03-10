import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .hri_bridge import HRIBridge
from .camera_parameters import RESOLUTION, get_parameters

class CameraCalibrator(Node):

    def __init__(self):
        super().__init__("camera_calibrator")

        self.get_logger().info("Starting camera calibrator")

        self.camera_matrix, self.camera_matrix_inv, self.dist_coeffs = get_parameters(
            RESOLUTION
        )
        self.subscriber = self.create_subscription(
            Image, "image_raw", self.frame_callback, 1
        )
        self.publisher = self.create_publisher(Image, "camera/color/image_raw", 1)

        self.br = HRIBridge()

    def frame_callback(self, frame_msg):
        frame = self.br.imgmsg_to_cv2(frame_msg, "bgr8")

        undistorted_image = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        flipped_undisorted_image = cv2.flip(undistorted_image, 1)
        self.publisher.publish(self.br.cv2_to_imgmsg(flipped_undisorted_image, "bgr8"))


def main(args=None):
    rclpy.init(args=args)

    camera_capturer = CameraCalibrator()

    rclpy.spin(camera_capturer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
