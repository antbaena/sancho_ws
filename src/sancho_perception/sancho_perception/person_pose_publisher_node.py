import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
import tf2_ros
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.duration import Duration

class PersonPosePublisher(Node):
    def __init__(self):
        super().__init__('person_pose_publisher')

        self.subscription = self.create_subscription(
            PoseArray,
            '/human_pose/keypoints3d',
            self.pose_array_callback,
            10
        )
        self.publisher_ = self.create_publisher(PoseStamped, 'person_pose', 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Nodo PersonPosePublisher iniciado.")

    def pose_array_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        # 1) Calcular centroide
        sum_x = sum_y = sum_z = 0.0
        for pose in msg.poses:
            sum_x += pose.position.x
            sum_y += pose.position.y
            sum_z += pose.position.z

        count = len(msg.poses)
        centroid_x = sum_x / count
        centroid_y = sum_y / count
        centroid_z = sum_z / count

        # 2) Construir PoseStamped en el frame original
        person_pose = PoseStamped()
        person_pose.header = msg.header
        person_pose.pose.position.x = centroid_x
        person_pose.pose.position.y = centroid_y
        person_pose.pose.position.z = centroid_z/1000.0  # Convertir a metros
        person_pose.pose.orientation.w = 1.0

        # 3) Transformar al frame "map"
        try:
            # IMPORTANTE: usar la misma marca de tiempo que llega en msg.header.stamp, si tu TF lo soporta
            transform = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp).to_msg(),  
                # Si tu TF no maneja muy bien el timestamp, podrías usar:
                # rclpy.time.Time().to_msg()
                timeout=Duration(seconds=0.5).to_msg()
            )

            # Aplica transform
            transformed_pose = tf2_geometry_msgs.do_transform_pose(person_pose, transform)
            self.publisher_.publish(transformed_pose)

            self.get_logger().info(
                f"Publicada pose en 'map': x={transformed_pose.pose.position.x:.2f}, "
                f"y={transformed_pose.pose.position.y:.2f}, z={transformed_pose.pose.position.z:.2f}"
            )

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
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
