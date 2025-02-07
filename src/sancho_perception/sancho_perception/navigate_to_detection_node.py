#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavigateToDetectionNode(Node):
    def __init__(self):
        super().__init__('navigate_to_detection_node')

        # Bandera para evitar enviar múltiples objetivos simultáneos.
        self.goal_active = False

        # Suscriptor a detecciones 3D
        self.subscription = self.create_subscription(
            PoseArray,
            '/human_pose/keypoints3d',
            self.keypoints_callback,
            10
        )
        self.get_logger().info("Nodo de navegación a detección 3D iniciado.")

        # Cliente de acción para Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def keypoints_callback(self, keypoints_msg: PoseArray):
        # Si ya se está procesando un objetivo, se ignoran nuevas detecciones
        if self.goal_active:
            self.get_logger().info("Objetivo en curso. Ignorando nueva detección.")
            return

        # Verificar que exista al menos un keypoint
        if not keypoints_msg.poses:
            self.get_logger().info("No se han recibido keypoints 3D.")
            return

        # Seleccionar un keypoint. En este ejemplo, se toma el primero.
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        # Se asigna el frame 'map'. Asegúrate de que coincida con el utilizado en tu Nav2.
        target_pose.header.frame_id = "map"
        target_pose.pose = keypoints_msg.poses[0]

        self.get_logger().info(f"Enviando objetivo de navegación: {target_pose.pose}")

        # Crear el mensaje de objetivo para la acción
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        # Marcar que ya hay un objetivo activo
        self.goal_active = True

        # Esperar a que el servidor de acción esté disponible y enviar el objetivo
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback de navegación: {feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Objetivo rechazado.")
            self.goal_active = False
            return

        self.get_logger().info("Objetivo aceptado. Ejecutando navegación...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Resultado de la navegación: {result}")
        self.goal_active = False  # Permite procesar nuevas detecciones una vez finalizado el objetivo

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navegación detenida por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
