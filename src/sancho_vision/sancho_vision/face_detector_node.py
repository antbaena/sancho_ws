import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sancho_msgs.msg import Face, FaceArray
from cv_bridge import CvBridge
import dlib
import cv2
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory

class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector')

        # Parámetros
        self.declare_parameter('image_topic', '/sancho_camera/image_rect')
        self.declare_parameter('detections_topic', '/face_detections')
        self.declare_parameter('use_cnn', False)
        self.declare_parameter('visualization', True)
        self.declare_parameter('processing_rate', 1.0)  # Hz

        model_path = os.path.join(
            get_package_share_directory('sancho_vision'),
            'models', 'mmod_human_face_detector.dat'
        )
        self.declare_parameter('model_path', model_path)

        # Leer parámetros
        image_topic      = self.get_parameter('image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        model_path       = self.get_parameter('model_path').get_parameter_value().string_value
        self.use_cnn          = self.get_parameter('use_cnn').get_parameter_value().bool_value
        self.visualization = self.get_parameter('visualization').get_parameter_value().bool_value
        rate_hz          = self.get_parameter('processing_rate').get_parameter_value().double_value

        # Preparar detector
        self.bridge = CvBridge()
        if self.use_cnn:
            if not os.path.exists(model_path):
                self.get_logger().error(f"No se encuentra el modelo en: {model_path}")
                raise FileNotFoundError(model_path)
            self.detector = dlib.cnn_face_detection_model_v1(model_path)
            self.get_logger().info("Usando detector CNN")
        else:
            self.detector = dlib.get_frontal_face_detector()
            self.get_logger().info("Usando detector frontal HOG")

        # Suscripción: solo actualiza self.latest_msg
        self.latest_msg = None
        self.sub = self.create_subscription(Image, image_topic,
                                           self._on_image_received, 10)

        # Publicadores
        self.pub = self.create_publisher(FaceArray, detections_topic, 10)
        if self.visualization:
            self.vis_pub = self.create_publisher(
                Image, detections_topic + '/visualization', 10)

        # Timer para procesar a tasa fija
        period_sec = 1.0 / max(rate_hz, 0.1)
        self.create_timer(period_sec, self._on_timer)

        self.get_logger().info(
            f"Procesando detección a {rate_hz:.1f} Hz (periodo {period_sec:.3f}s)"
        )
        self.get_logger().info(
            f"Suscrito a {image_topic}, publicando en {detections_topic}"
        )

    def _on_image_received(self, msg: Image):
        # Guardamos siempre la última imagen recibida
        self.latest_msg = msg

    def _on_timer(self):
        if self.latest_msg is None:
            return  # aún no hemos recibido nada

        msg = self.latest_msg
        self.latest_msg = None  # opcional: procesar cada imagen solo una vez

        # Convertir a OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image_copy = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        if self.use_cnn:
            raw_detections = self.detector(cv_image, 1)
            detections = [(d.rect, float(d.confidence)) for d in raw_detections]
        else:
            raw_detections = self.detector(cv_image, 1)
            detections = [(d, 1.0) for d in raw_detections]

        face_array = FaceArray()
        face_array.header = msg.header

        for rect, conf in detections:
            cx = (rect.left() + rect.right()) / 2.0
            cy = (rect.top() + rect.bottom()) / 2.0
            p = Point(x=cx, y=cy, z=0.0)
            w = float(rect.right() - rect.left())
            h = float(rect.bottom() - rect.top())

            f = Face(header=msg.header,
                     center=p, width=w, height=h, confidence=conf)
            face_array.faces.append(f)

            if self.visualization:
                cv2.rectangle(cv_image_copy,
                              (rect.left(), rect.top()),
                              (rect.right(), rect.bottom()),
                              (0,255,0), 2)
                cv2.putText(cv_image_copy, f"{conf:.2f}",
                            (rect.left(), rect.top()-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Publicar resultados
        self.pub.publish(face_array)
        self.get_logger().info(f"Publicadas {len(face_array.faces)} caras")

        if self.visualization:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(cv_image_copy, "bgr8")
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)
            except Exception as e:
                self.get_logger().error(f"Error al convertir imagen vis: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
