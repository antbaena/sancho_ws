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

        # Suscripción a la imagen de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Tópico de la cámara
            self.image_callback,
            10
        )

        # Publicadores: uno para los keypoints y otro para la imagen anotada
        self.skeleton_pub = self.create_publisher(PoseArray, '/human_pose/keypoints2d', 10)
        self.annotated_pub = self.create_publisher(Image, '/human_pose/annotated_image', 10)

        self.bridge = CvBridge()

        # Inicializamos MediaPipe Pose y sus utilidades de dibujo
        self.mp_pose = mp.solutions.pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils

    def image_callback(self, msg):
        # Convertir el mensaje ROS a imagen OpenCV (BGR)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error al convertir la imagen: {}".format(e))
            return

        # Convertir la imagen de BGR a RGB para procesarla con MediaPipe
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.mp_pose.process(rgb_image)

        # Crear el mensaje PoseArray y asignar el encabezado de la imagen recibida
        pose_array = PoseArray()
        pose_array.header = msg.header

        # Si se han detectado landmarks, procesarlos y dibujarlos
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                pose = Pose()
                pose.position.x = landmark.x
                pose.position.y = landmark.y
                pose.position.z = landmark.z  # Se incluye la coordenada z si es de interés
                # La orientación queda sin definir, ya que MediaPipe no la proporciona
                pose_array.poses.append(pose)

            # Crear una copia de la imagen para dibujar los landmarks
            annotated_image = cv_image.copy()

            # Dibujar los landmarks y las conexiones en la imagen
            self.mp_drawing.draw_landmarks(
                image=annotated_image,
                landmark_list=results.pose_landmarks,
                connections=mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2)
            )
        else:
            annotated_image = cv_image  # Si no hay detección, se publica la imagen original

        # Publicar el mensaje PoseArray con los keypoints
        self.skeleton_pub.publish(pose_array)

        # Convertir la imagen anotada a mensaje ROS y publicarla
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header  # Mantener el mismo encabezado
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error("Error al convertir la imagen anotada: {}".format(e))

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
