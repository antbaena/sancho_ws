import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, TimeSynchronizer

class YUYVToRGBConverter(Node):
    def __init__(self):
        super().__init__('yuyv_to_rgb_converter')
        self.bridge = CvBridge()

        # Suscriptores
        self.image_sub = Subscriber(self, Image, '/image_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/camera_info')

        # Sincronizador
        self.ts = TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.ts.registerCallback(self.image_callback)

        self.publisher = self.create_publisher(Image, '/camera/color/image_rgb', 10)

    def image_callback(self, img_msg, info_msg):
        self.get_logger().info(f'Formato de la imagen recibida: {img_msg.encoding}')
        yuyv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        # Convertir la imagen de formato YUYV a RGB
        rgb_image = cv2.cvtColor(yuyv_image, cv2.COLOR_YUV2RGB_YUY2)

        # Convertir la imagen OpenCV de nuevo al formato ROS
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        rgb_msg.header = img_msg.header  # Asegurarse de que la cabecera sea la misma
        self.publisher.publish(rgb_msg)

def main(args=None):
    rclpy.init(args=args)
    converter = YUYVToRGBConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
