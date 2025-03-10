#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class RoamingNode(Node):
    def __init__(self):
        super().__init__('roaming_node')
        
        # Declarar parámetros configurables
        self.declare_parameter('min_x', -5.0)
        self.declare_parameter('max_x', 5.0)
        self.declare_parameter('min_y', -5.0)
        self.declare_parameter('max_y', 5.0)
        self.declare_parameter('idle_time_before_new_goal', 2.0)
        self.declare_parameter('check_interval', 5.0)
        
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.idle_time = self.get_parameter('idle_time_before_new_goal').value
        self.timer_period = self.get_parameter('check_interval').value

        # Timer para verificar el estado del robot
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Cliente de acción para NavigateToPose de Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Estado de la meta activa y tiempo de la última meta
        self.goal_active = False
        self.last_goal_end_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        time_since_goal = (current_time - self.last_goal_end_time).nanoseconds / 1e9
        # Solo genera una nueva meta si no hay meta activa y se ha cumplido el tiempo de inactividad
        if not self.goal_active and time_since_goal >= self.idle_time:
            self.get_logger().info('Robot inactivo, generando nueva meta de roaming...')
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.generate_random_pose()
            self.send_goal(goal_msg)

    def generate_random_pose(self):
        # Genera un PoseStamped aleatorio dentro de los límites configurados
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = random.uniform(self.min_x, self.max_x)
        pose.pose.position.y = random.uniform(self.min_y, self.max_y)
        pose.pose.position.z = 0.0
        
        # Generar un ángulo aleatorio (yaw) y convertirlo a cuaternión
        yaw = random.uniform(-math.pi, math.pi)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        self.get_logger().info(
            f'Nueva meta generada: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, yaw={yaw:.2f}'
        )
        return pose

    def send_goal(self, goal_msg):
        # Espera a que el servidor de acción esté disponible
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor de acción "navigate_to_pose" no disponible.')
            return
        
        self.goal_active = True
        self.get_logger().info('Enviando meta de navegación...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Meta rechazada por el servidor de navegación.')
            self.goal_active = False
            self.last_goal_end_time = self.get_clock().now()
            return
        
        self.get_logger().info('Meta aceptada, esperando resultado...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Procesamiento opcional del feedback recibido
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Feedback recibido: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().warn('Navegación abortada. Se generará una nueva meta.')
        elif status != 0:
            self.get_logger().warn(f'Navegación finalizada con estado: {status}')
        else:
            self.get_logger().info('Meta alcanzada exitosamente.')
        
        self.goal_active = False
        self.last_goal_end_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = RoamingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado, cerrando nodo de roaming...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
