import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from rclpy.time import Time

class PersonPosePublisher(Node):
    def __init__(self):
        super().__init__('person_pose_publisher')

        # Suscripción a PoseArray
        self.subscription = self.create_subscription(
            PoseArray,
            '/human_pose/keypoints3d',
            self.pose_array_callback,
            10
        )

        # Publicación de PoseStamped transformado a "map"
        self.publisher_ = self.create_publisher(PoseStamped, 'person_pose', 10)

        # TF2 Buffer y Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Nodo PersonPosePublisher iniciado.")

    def pose_array_callback(self, msg: PoseArray):
        # Verificar que haya keypoints
        if not msg.poses:
            return

        # Calcular el centroide
        sum_x = sum_y = sum_z = 0.0
        for pose in msg.poses:
            sum_x += pose.position.x
            sum_y += pose.position.y
            sum_z += pose.position.z

        count = len(msg.poses)
        centroid_x = sum_x / count
        centroid_y = sum_y / count
        centroid_z = sum_z / count

        # Construir el PoseStamped en el frame original
        person_pose = PoseStamped()
        person_pose.header.stamp = msg.header.stamp  # puedes usar también Time() si hay problemas de sincronía
        person_pose.header.frame_id = msg.header.frame_id
        person_pose.pose.position.x = centroid_x
        person_pose.pose.position.y = centroid_y
        person_pose.pose.position.z = centroid_z
        person_pose.pose.orientation.w = 1.0
        
        # Transformar al frame "map"
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="map",
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time().to_msg(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )


            # OJO: Primer argumento = PoseStamped, segundo = TransformStamped
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(person_pose, transform)

            self.publisher_.publish(transformed_pose)

            self.get_logger().info(
                f"Publicada pose en 'map': "
                f"x={transformed_pose.pose.position.x:.2f}, "
                f"y={transformed_pose.pose.position.y:.2f}, "
                f"z={transformed_pose.pose.position.z:.2f}"
            )

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f"TF no disponible: {e}")
            return

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
