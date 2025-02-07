#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped

class PersonPosePublisher(Node):
    def __init__(self):
        super().__init__('person_pose_publisher')
        # Suscriptor al PoseArray con los keypoints 3D
        self.subscription = self.create_subscription(
            PoseArray,
            '/human_pose/keypoints3d',
            self.pose_array_callback,
            10)
        # Publicador para la posición de la persona (PoseStamped)
        self.publisher_ = self.create_publisher(PoseStamped, 'person_pose', 10)
        self.get_logger().info("Nodo PersonPosePublisher iniciado.")

    def pose_array_callback(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warning("PoseArray recibido sin keypoints.")
            return

        # Calcular el centroide de los keypoints.
        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0
        count = 0
        for pose in msg.poses:
            sum_x += pose.position.x
            sum_y += pose.position.y
            sum_z += pose.position.z
            count += 1

        if count == 0:
            self.get_logger().warning("No se pudieron calcular keypoints válidos.")
            return

        centroid_x = sum_x / count
        centroid_y = sum_y / count
        centroid_z = sum_z / count

        # Crear y llenar el mensaje PoseStamped
        person_pose = PoseStamped()
        # Usamos el header del mensaje recibido, pero nos aseguramos que el frame sea "map"
        person_pose.header = msg.header
        person_pose.header.frame_id = "map"
        person_pose.pose.position.x = centroid_x
        person_pose.pose.position.y = centroid_y
        person_pose.pose.position.z = centroid_z

        # Orientación: aquí se asigna la identidad (sin rotación), pero puedes ajustar si es necesario.
        person_pose.pose.orientation.x = 0.0
        person_pose.pose.orientation.y = 0.0
        person_pose.pose.orientation.z = 0.0
        person_pose.pose.orientation.w = 1.0

        self.publisher_.publish(person_pose)
        self.get_logger().info(
            f"Publicada posición de la persona: ({centroid_x:.2f}, {centroid_y:.2f}, {centroid_z:.2f})"
        )

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
