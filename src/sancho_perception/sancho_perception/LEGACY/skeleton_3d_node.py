import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Skeleton3DNode(Node):
    def __init__(self):
        super().__init__('skeleton_3d_node')

        self.bridge = CvBridge()
        self.intrinsic_matrix = None

        # Suscriptores sincronizados
        self.sub_keypoints_2d = Subscriber(self, PoseArray, '/human_pose/keypoints2d')
        self.sub_depth = Subscriber(self, Image, '/camera/depth/image_raw')
        self.sub_info = Subscriber(self, CameraInfo, '/camera/depth/camera_info')

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_keypoints_2d, self.sub_depth, self.sub_info],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # Publicadores
        self.publisher_3d = self.create_publisher(PoseArray, '/human_pose/keypoints3d', 10)
        self.annotated_pub = self.create_publisher(Image, '/human_pose/annotated_depth', 10)

    def sync_callback(self, keypoints_2d_msg, depth_msg, info_msg):
        # 1) Inicializar matriz intrínseca
        if self.intrinsic_matrix is None:
            self.intrinsic_matrix = np.array(info_msg.k).reshape(3, 3)
            self.get_logger().info(f"Matriz intrínseca inicializada:\n{self.intrinsic_matrix}")

        # 2) Convertir depth a OpenCV
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen de profundidad: {e}")
            return

        # 3) Preparar PoseArray para keypoints 3D
        pose_array_3d = PoseArray()
        pose_array_3d.header = keypoints_2d_msg.header

        # 4) Crear una copia de la depth_image para anotaciones
        try:
            if depth_image.dtype != np.uint8:
                depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_uint8 = depth_norm.astype(np.uint8)
            else:
                depth_uint8 = depth_image
            annotated_image = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
        except Exception as e:
            self.get_logger().error(f"Error preparando imagen de profundidad: {e}")
            annotated_image = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)

        # 5) Calcular 3D de cada keypoint 2D
        height, width = depth_image.shape[:2]
        for idx, pose_2d in enumerate(keypoints_2d_msg.poses):
            x_norm = pose_2d.position.x
            y_norm = pose_2d.position.y

            # Convertir coordenadas normalizadas (0..1) a píxeles
            x_2d = int(x_norm * width)
            y_2d = int(y_norm * height)

            if 0 <= x_2d < width and 0 <= y_2d < height:
                depth_value = depth_image[y_2d, x_2d]

                # Salta si la profundidad es 0 o NaN
                if depth_value <= 0:
                    continue

                # Proyección a 3D
                cx = self.intrinsic_matrix[0, 2]
                cy = self.intrinsic_matrix[1, 2]
                fx = self.intrinsic_matrix[0, 0]
                fy = self.intrinsic_matrix[1, 1]

                # X = (u - cx) * Z / fx
                # Y = (v - cy) * Z / fy
                # Z = depth_value
                X = (x_2d - cx) * depth_value / fx
                Y = (y_2d - cy) * depth_value / fy
                Z = depth_value

                # Crear Pose con la posición 3D
                pose_3d = Pose()
                pose_3d.position.x = float(X)
                pose_3d.position.y = float(Y)
                pose_3d.position.z = float(Z)
                pose_array_3d.poses.append(pose_3d)

                # Anotar en la imagen
                cv2.circle(annotated_image, (x_2d, y_2d), 4, (0, 255, 0), -1)
                cv2.putText(annotated_image, f"{idx}", (x_2d+5, y_2d-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 6) Publicar keypoints 3D
        self.publisher_3d.publish(pose_array_3d)

        # 7) Publicar imagen de profundidad anotada
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = depth_msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Error publicando imagen anotada: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Skeleton3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
