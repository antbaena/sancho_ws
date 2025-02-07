import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
import open3d as o3d  # Opcional: para futuras expansiones, por ejemplo, para generar nubes de puntos

class Skeleton3DNode(Node):
    def __init__(self):
        super().__init__('skeleton_3d_node')
        
        # Inicializamos el puente para convertir entre mensajes ROS y OpenCV
        self.bridge = CvBridge()
        self.intrinsic_matrix = None
        
        # Suscriptores sincronizados para los keypoints 2D, la imagen de profundidad y la info de la cámara
        self.sub_keypoints_2d = Subscriber(self, PoseArray, '/human_pose/keypoints2d')
        self.sub_depth = Subscriber(self, Image, '/camera/depth/image_raw')
        self.sub_info = Subscriber(self, CameraInfo, '/camera/depth/camera_info')
        
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_keypoints_2d, self.sub_depth, self.sub_info],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publicador para los keypoints 3D
        self.publisher_3d = self.create_publisher(PoseArray, '/human_pose/keypoints3d', 10)
        # Publicador para la imagen de profundidad anotada (para visualización)
        self.annotated_pub = self.create_publisher(Image, '/human_pose/annotated_depth', 10)
    
    def sync_callback(self, keypoints_2d_msg, depth_msg, info_msg):
        # Inicializar la matriz intrínseca si aún no se ha hecho
        if self.intrinsic_matrix is None:
            self.intrinsic_matrix = np.array(info_msg.k).reshape(3, 3)
            self.get_logger().info("Matriz intrínseca inicializada:\n{}".format(self.intrinsic_matrix))
        
        # Convertir la imagen de profundidad a formato OpenCV
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error("Error al convertir la imagen de profundidad: {}".format(e))
            return

        # Crear el mensaje PoseArray para los keypoints 3D
        pose_array_3d = PoseArray()
        pose_array_3d.header = keypoints_2d_msg.header

        # Preparar la imagen de profundidad para su visualización:
        # Normalizamos la imagen y aplicamos un colormap para obtener una imagen BGR
        try:
            if depth_image.dtype != np.uint8:
                depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_uint8 = depth_norm.astype(np.uint8)
            else:
                depth_uint8 = depth_image.copy()
            annotated_image = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
        except Exception as e:
            self.get_logger().error("Error al preparar la imagen de profundidad para anotación: {}".format(e))
            annotated_image = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)

        # Procesar cada keypoint 2D y calcular su posición 3D
        for idx, pose in enumerate(keypoints_2d_msg.poses):
            # Convertir las coordenadas normalizadas (0 a 1) a píxeles
            x_2d = int(pose.position.x * depth_image.shape[1])
            y_2d = int(pose.position.y * depth_image.shape[0])
            
            if 0 <= x_2d < depth_image.shape[1] and 0 <= y_2d < depth_image.shape[0]:
                depth_value = depth_image[y_2d, x_2d]
                # Si no se dispone de un valor de profundidad (0), se omite el keypoint
                if depth_value == 0:
                    continue

                # Calcular las coordenadas 3D:
                # X = (u - cx) * Z / fx ; Y = (v - cy) * Z / fy ; Z = profundidad
                x_3d = (x_2d - self.intrinsic_matrix[0, 2]) * depth_value / self.intrinsic_matrix[0, 0]
                y_3d = (y_2d - self.intrinsic_matrix[1, 2]) * depth_value / self.intrinsic_matrix[1, 1]
                z_3d = depth_value

                # Crear el mensaje Pose y asegurarse de que los valores sean float nativos
                pose_3d = Pose()
                pose_3d.position.x = float(x_3d)
                pose_3d.position.y = float(y_3d)
                pose_3d.position.z = float(z_3d)
                # La orientación queda sin definir (se mantienen valores por defecto)
                pose_array_3d.poses.append(pose_3d)
                
                # Anotar la imagen de profundidad: dibujar un círculo y el índice del keypoint
                cv2.circle(annotated_image, (x_2d, y_2d), 4, (0, 255, 0), -1)
                cv2.putText(annotated_image, f"{idx}", (x_2d + 5, y_2d - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                self.get_logger().warning(f"Keypoint {idx} fuera de rango: ({x_2d}, {y_2d})")
        
        # Publicar los keypoints 3D
        self.publisher_3d.publish(pose_array_3d)
        
        # Convertir y publicar la imagen de profundidad anotada
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = depth_msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error("Error al publicar la imagen anotada: {}".format(e))

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
