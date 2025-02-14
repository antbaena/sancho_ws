import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from rclpy.duration import Duration

class PersonPosePublisher(Node):
    def __init__(self):
        super().__init__('person_pose_publisher')

        # Suscripción a PoseArray con los keypoints 3D
        self.subscription = self.create_subscription(
            PoseArray,
            '/human_pose/keypoints3d',
            self.pose_array_callback,
            10
        )
        # Publicación de la pose centrada
        self.publisher_ = self.create_publisher(PoseStamped, 'person_pose', 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Nodo PersonPosePublisher iniciado.")

        # (Opcional) Parámetros de outlier
        # Máxima distancia permitida a la media preliminar para no descartar el punto
        self.max_outlier_dist = 0.5  # en metros

        # (Opcional) Keypoints de torso más fiables, según índice de MediaPipe:
        #  - 11: left_shoulder
        #  - 12: right_shoulder
        #  - 23: left_hip
        #  - 24: right_hip
        self.torso_indices = [11, 12, 23, 24]

        # (Opcional) Filtro temporal: guardamos la pose anterior
        self.prev_pose = None
        # Ganancia de suavizado: 0.0 = sin suavizado, 0.5 = semi-lento
        self.alpha = 0.3

    def pose_array_callback(self, msg: PoseArray):
        # 1. Recolectar keypoints 3D válidos
        valid_points = []
        for idx, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            # a) Usar sólo ciertos keypoints del torso (opcional)
            if idx not in self.torso_indices:
                continue

            # b) Comprobar si la profundidad es válida
            #    (por ejemplo, z <= 0 indica sin valor o error)
            if z <= 0 or z > 10.0:  
                # Descarta valores de profundidad imposibles o absurdos (>10m si no esperas a gente tan lejos)
                continue

            valid_points.append([x, y, z])

        if not valid_points:
            return  # No hay puntos válidos, no se publica nada

        # Convertir a numpy
        pts = np.array(valid_points)  # shape (N, 3)

        # 2. Calcular una media preliminar
        mean_pre = np.mean(pts, axis=0)  # (x_mean, y_mean, z_mean)

        # 3. Filtrar outliers: descartar puntos demasiado alejados de la media
        dists = np.linalg.norm(pts - mean_pre, axis=1)
        inliers = pts[dists < self.max_outlier_dist]

        if len(inliers) == 0:
            # Si todos quedaron fuera, usar la media preliminar o saltar
            inliers = pts

        # 4. Calcular la media final
        center_3d = np.mean(inliers, axis=0)

        # (Opcional) Filtro temporal: suaviza con la pose previa
        # Se asume prev_pose también en 3D
        if self.prev_pose is not None:
            center_3d = self.alpha * center_3d + (1.0 - self.alpha) * self.prev_pose
        self.prev_pose = center_3d

        # 5. Construir PoseStamped en el frame actual (msg.header.frame_id)
        person_pose = PoseStamped()
        person_pose.header = msg.header  # conserva stamp y frame_id
        person_pose.pose.position.x = float(center_3d[0])
        person_pose.pose.position.y = float(center_3d[1])
        person_pose.pose.position.z = float(center_3d[2])/1000.0  # Convertir a metros
        person_pose.pose.orientation.w = 1.0  # sin orientación

        # 6. Transformar la pose al frame "map"
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                rclpy.time.Time().to_msg(),
                timeout=Duration(seconds=0.5).to_msg()
            )

            # Aplicar transform
            transformed_pose = tf2_geometry_msgs.do_transform_pose(person_pose, transform)
            self.publisher_.publish(transformed_pose)

            self.get_logger().info(
                f"Publicada pose en 'map': "
                f"x={transformed_pose.pose.position.x:.2f}, "
                f"y={transformed_pose.pose.position.y:.2f}, "
                f"z={transformed_pose.pose.position.z:.2f}"
            )

        except Exception as e:
            self.get_logger().warning(f"TF no disponible: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PersonPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
