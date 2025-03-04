import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class SkeletonDetectionNode(Node):
    def __init__(self):
        super().__init__('skeleton_detection_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.skeleton_pub = self.create_publisher(PoseArray, '/human_pose/keypoints2d', 10)
        self.annotated_pub = self.create_publisher(Image, '/human_pose/annotated_image', 10)

        self.bridge = CvBridge()

        # MediaPipe
        self.mp_pose = mp.solutions.pose.Pose(
            # Puedes configurar aquí algunos parámetros, p.e.:
            static_image_mode=False,       # Si se procesa video en tiempo real
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def image_callback(self, msg):
        # 1) Convertir la imagen ROS a formato OpenCV (BGR)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # 2) Procesar en RGB con Mediapipe
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.mp_pose.process(rgb_image)

        # 3) Crear PoseArray
        pose_array = PoseArray()
        pose_array.header = msg.header  # Mantener stamp y frame_id de la cámara

        # 4) Extraer landmarks si existen
        annotated_image = cv_image.copy()  # Para dibujar
        if results.pose_landmarks is not None:
            for landmark in results.pose_landmarks.landmark:
                pose = Pose()
                pose.position.x = landmark.x
                pose.position.y = landmark.y
                pose.position.z = landmark.z
                pose_array.poses.append(pose)

            # Dibuja pose en la imagen
            self.mp_drawing.draw_landmarks(
                image=annotated_image,
                landmark_list=results.pose_landmarks,
                connections=mp.solutions.pose.POSE_CONNECTIONS
            )

        # 5) Publicar keypoints 2D
        self.skeleton_pub.publish(pose_array)

        # 6) Publicar imagen anotada
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Error publicando imagen anotada: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SkeletonDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
