import cv2
import requests
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .hri_bridge import HRIBridge


class IPCameraCapturer(Node):

    def __init__(self, url):
        """Initializes the camera capturer, creates the video capture object and the publisher object.

        Args:
            url (str): URL of the source of images.
        """

        super().__init__("ip_camera_capturer")

        self.url = url
        self.publisher = self.create_publisher(Image, "camera/color/image_raw", 10)
        self.br = HRIBridge()

        self.run()

    def run(self):
        """Loop that capture frames and publish them on the corresponding topic."""

        i = 0
        while True:
            img_resp = requests.get(self.url)
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            img = cv2.imdecode(img_arr, -1)
            img = cv2.resize(img, (int(img.shape[1] / 2), int(img.shape[0] / 2)))

            if i == 0:
                self.get_logger().info("Camera initialized successfully")

            self.publisher.publish(self.br.cv2_to_imgmsg(img))

            i += 1


def main(args=None):
    rclpy.init(args=args)

    ip_camera_capturer = IPCameraCapturer("http://192.168.1.153:8080/shot.jpg")

    rclpy.spin(ip_camera_capturer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
