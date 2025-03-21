import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .hri_bridge import HRIBridge


class CameraCapturer(Node):

    def __init__(self, cam_index=0):
        """Initializes the camera capturer, creates the video capture object and the publisher object.

        Args:
            cam_index (int): Index of the camera.
        """

        super().__init__("camera_capturer")

        self.get_logger().info(
            "Trying to start camera on index " + str(cam_index) + ".."
        )
        self.capture = cv2.VideoCapture(cam_index)

        self.publisher = self.create_publisher(Image, "camera/color/image_raw", 1)
        self.publisher_disorted = self.create_publisher(
            Image, "camera/color/disorted_image_raw", 1
        )

        self.br = HRIBridge()

        self.run()

    def run(self):
        """Loop that capture frames and publish them on the corresponding topic."""

        i = 0
        while True:
            ret, frame = self.capture.read()

            if not ret:
                break

            if i == 0:
                self.get_logger().info("Camera initializated succesfully")

            self.publisher.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

            i += 1

            cv2.waitKey(1)

        self.capture.release()


def main(args=None):
    rclpy.init(args=args)

    camera_capturer = CameraCapturer(0)

    rclpy.spin(camera_capturer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
