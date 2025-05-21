import os
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Image
from sancho_msgs.msg import Face, FaceArray
from cv_bridge import CvBridge
import dlib
import cv2
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State

class FaceDetectorLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('face_detector_lifecycle')
        # Declaración de parámetros
        self.declare_parameter('image_topic', '/sancho_camera/image_rect')
        self.declare_parameter('detections_topic', '/face_detections')
        self.declare_parameter('use_cnn', False)
        self.declare_parameter('visualization', True)
        self.declare_parameter('processing_rate', 1.0)  # Hz

        default_model = os.path.join(
            get_package_share_directory('sancho_vision'),
            'models', 'mmod_human_face_detector.dat'
        )
        self.declare_parameter('model_path', default_model)

        # Variables de estado
        self.bridge = CvBridge()
        self.detector = None
        self.sub = None
        self.pub = None
        self.vis_pub = None
        self.timer = None
        self.latest_msg = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configurando FaceDetectorLifecycleNode...')
        # Lectura de parámetros
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.use_cnn = self.get_parameter('use_cnn').get_parameter_value().bool_value
        self.visualization = self.get_parameter('visualization').get_parameter_value().bool_value
        rate_hz = self.get_parameter('processing_rate').get_parameter_value().double_value

        # Inicializar detector
        if self.use_cnn:
            if not os.path.exists(model_path):
                self.get_logger().error(f"No se encuentra el modelo en: {model_path}")
                return TransitionCallbackReturn.FAILURE
            self.detector = dlib.cnn_face_detection_model_v1(model_path)
            self.get_logger().info('Detector CNN configurado')
        else:
            self.detector = dlib.get_frontal_face_detector()
            self.get_logger().info('Detector HOG configurado')

        # Crear suscriptor y publicadores
        
        self.pub = self.create_lifecycle_publisher(FaceArray, detections_topic, 10)
        if self.visualization:
            self.vis_pub = self.create_lifecycle_publisher(
                Image, detections_topic + '/visualization', 10)

        # Timer para procesado periódico
        period_sec = 1.0 / max(rate_hz, 0.1)
        self.timer = self.create_timer(period_sec, self._on_timer)

        self.get_logger().info(
            f"Configurado: suscrito a {self.image_topic}, publicando en {detections_topic} a {rate_hz:.1f}Hz"
        )
        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activando FaceDetectorLifecycleNode...')
        self.sub = self.create_subscription(
            Image, self.image_topic, self._on_image_received, 10)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Desactivando FaceDetectorLifecycleNode...')
        if self.sub:
            self.destroy_subscription(self.sub)
            self.sub = None
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub:
            self.destroy_lifecycle_publisher(self.pub)
            self.pub = None
        if self.vis_pub:
            self.destroy_lifecycle_publisher(self.vis_pub)
            self.vis_pub = None
        if self.detector:
            self.detector = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Limpiando recursos de FaceDetectorLifecycleNode...')
        # Destruir suscriptor, publicadores y timer
        if self.sub:
            self.destroy_subscription(self.sub)
            self.sub = None
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub:
            self.destroy_lifecycle_publisher(self.pub)
            self.pub = None
        if self.vis_pub:
            self.destroy_lifecycle_publisher(self.vis_pub)
            self.vis_pub = None
        if self.detector:
            self.detector = None
        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('FaceDetectorLifecycleNode apagándose...')
        return super().on_shutdown(state)

    # Callbacks útiles
    def _on_image_received(self, msg: Image):
        self.latest_msg = msg

    def _on_timer(self):
        if self.latest_msg is None:
            return
        msg = self.latest_msg
        self.latest_msg = None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image_copy = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        if self.use_cnn:
            raw = self.detector(cv_image, 1)
            dets = [(d.rect, float(d.confidence)) for d in raw]
        else:
            raw = self.detector(cv_image, 1)
            dets = [(d, 1.0) for d in raw]

        face_array = FaceArray()
        face_array.header = msg.header
        for rect, conf in dets:
            cx = (rect.left() + rect.right()) / 2.0
            cy = (rect.top() + rect.bottom()) / 2.0
            p = Point(x=cx, y=cy, z=0.0)
            w = float(rect.right() - rect.left())
            h = float(rect.bottom() - rect.top())
            f = Face(header=msg.header, center=p, width=w, height=h, confidence=conf)
            face_array.faces.append(f)
            if self.visualization:
                cv2.rectangle(cv_image_copy,
                              (rect.left(), rect.top()),
                              (rect.right(), rect.bottom()),
                              (0,255,0), 2)
                cv2.putText(cv_image_copy, f"{conf:.2f}",
                            (rect.left(), rect.top()-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        self.get_logger().info(str(face_array))
        self.pub.publish(face_array)
        self.get_logger().info(f"Publicado {len(face_array.faces)} caras")
        if self.visualization:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(cv_image_copy, 'bgr8')
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)
            except Exception as e:
                self.get_logger().error(f"Error al publicar visualización: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
