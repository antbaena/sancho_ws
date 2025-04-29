#!/usr/bin/env python3
"""
Nodo de ROS2 (Humble) mejorado para roaming aleatorio del robot usando Nav2.
Incluye validación de path mediante acción ComputePathToPose y generación de objetivos
relativos a la posición actual dentro de una ventana local.
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from action_msgs.msg import GoalStatus

import random
import math

class RoamingNode(Node):
    def __init__(self):
        super().__init__('roaming_node')
        self.callback_group = ReentrantCallbackGroup()

        # Parámetros configurables
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_local_window', True)
        self.declare_parameter('local_range_x', 4.5)
        self.declare_parameter('local_range_y', 4.5)
        self.declare_parameter('max_generate_attempts', 5)
        self.declare_parameter('recent_goal_history', 10)
        self.declare_parameter('idle_time_before_new_goal', 2.0)
        self.declare_parameter('check_interval', 5.0)
        self.declare_parameter('max_path_length', 10.0)
        self.declare_parameter('compute_path_timeout', 5.0)

        # Obtener parámetros
        self.frame_id = self.get_parameter('frame_id').value
        self.use_local_window = self.get_parameter('use_local_window').value
        self.local_range_x = self.get_parameter('local_range_x').value
        self.local_range_y = self.get_parameter('local_range_y').value
        self.max_attempts = int(self.get_parameter('max_generate_attempts').value)
        self.history_size = int(self.get_parameter('recent_goal_history').value)
        self.idle_time = self.get_parameter('idle_time_before_new_goal').value
        self.timer_period = self.get_parameter('check_interval').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.compute_path_timeout = self.get_parameter('compute_path_timeout').value

        # Estado
        self.recent_goals = []
        self.goal_active = False
        self.last_goal_end_time = self.get_clock().now()

        # TF2 listener
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Action clients
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group)
        self.path_action_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose', callback_group=self.callback_group)

        # Timer para disparar generación
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback, callback_group=self.callback_group)

    def timer_callback(self):
        idle_elapsed = (self.get_clock().now() - self.last_goal_end_time).nanoseconds / 1e9
        if not self.goal_active and idle_elapsed >= self.idle_time:
            self.get_logger().info('Robot inactivo: generando nueva meta...')
            pose = self._generate_valid_random_pose()
            if pose:
                goal = NavigateToPose.Goal()
                goal.pose = pose
                self._send_goal(goal)
            else:
                self.get_logger().warn('No se generó pose válida tras varios intentos.')

    def _generate_valid_random_pose(self):
        # Obtener pose actual
        try:
            # primero esperas a que exista un transform válido
            if not self.tf_buffer.can_transform(
                self.frame_id, 'base_link',
                rclpy.time.Time(),  # tiempo = 0 → último disponible
                timeout=Duration(seconds=self.compute_path_timeout)
            ):
                self.get_logger().warn('No hay transform map→base_link válido aún.')
                return None
            # y luego pides la última transformación
            trans = self.tf_buffer.lookup_transform(
                self.frame_id, 'base_link',
                rclpy.time.Time(),  # solicita el último transform
                timeout=Duration(seconds=self.compute_path_timeout)
            )
            start = PoseStamped()
            start.header.frame_id = self.frame_id
            start.header.stamp = self.get_clock().now().to_msg()
            start.pose.position.x = trans.transform.translation.x
            start.pose.position.y = trans.transform.translation.y
            start.pose.position.z = trans.transform.translation.z
            start.pose.orientation = trans.transform.rotation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Error al obtener pose actual: {e}')
            return None

        for _ in range(self.max_attempts):
            # Generación local o global según parámetro
            if self.use_local_window:
                # Ventana relativa
                min_x = start.pose.position.x - self.local_range_x
                max_x = start.pose.position.x + self.local_range_x
                min_y = start.pose.position.y - self.local_range_y
                max_y = start.pose.position.y + self.local_range_y
            else:
                # Ventana global configurable (mantener min/max originales si se desean)
                min_x = self.get_parameter('frame_id').value  # placeholder si se implementa global
                max_x = min_x
                min_y = self.get_parameter('frame_id').value
                max_y = min_y
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            yaw = random.uniform(-math.pi, math.pi)

            goal = PoseStamped()
            goal.header.frame_id = self.frame_id
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            key = (round(x, 2), round(y, 2))
            if key in self.recent_goals:
                continue

            # Calcular ruta para validar
            if not self.path_action_client.wait_for_server(timeout_sec=self.compute_path_timeout):
                self.get_logger().error('Action server compute_path_to_pose no disponible.')
                return None
            path_goal = ComputePathToPose.Goal()


            # calcular ángulo del robot hacia la meta
            dx = goal.pose.position.x - start.pose.position.x
            dy = goal.pose.position.y - start.pose.position.y
            yaw_to_goal = math.atan2(dy, dx)

            # actualizar la orientación del "start" pose
            q = quaternion_from_euler(0, 0, yaw_to_goal)
            start.pose.orientation.x = q[0]
            start.pose.orientation.y = q[1]
            start.pose.orientation.z = q[2]
            start.pose.orientation.w = q[3]
            
            goal.pose.orientation = start.pose.orientation

            path_goal.start = start
            path_goal.goal = goal
            send_fut = self.path_action_client.send_goal_async(path_goal)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=self.compute_path_timeout)
            if not send_fut.done():
                self.get_logger().warn('Timeout al enviar ComputePathToPose.')
                continue
            handle = send_fut.result()
            if not handle.accepted:
                self.get_logger().warn('ComputePathToPose rechazado.')
                continue
            result_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_fut, timeout_sec=self.compute_path_timeout)
            path = result_fut.result().result.path

            num_segments = len(path.poses)
            self.get_logger().debug(f'Path segments: {num_segments}')
            if num_segments < 2:
                self.get_logger().info('Pose descartada: sin trayectoria válida.')
                continue

            length = 0.0
            for i in range(num_segments - 1):
                dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x
                dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y
                length += math.hypot(dx, dy)

            if length <= 0.0 or length > self.max_path_length:
                self.get_logger().info(f'Pose descartada: longitud inválida ({length:.2f}m)')
                continue

            # Aceptar meta válida
            self.recent_goals.append(key)
            if len(self.recent_goals) > self.history_size:
                self.recent_goals.pop(0)
            self.get_logger().info(
                f'Pose aceptada: x={x:.2f}, y={y:.2f}, length={length:.2f}m dentro de ventana local'
            )
            return goal
        return None

    def _send_goal(self, goal_msg):
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server navigate_to_pose no disponible.')
            return
        self.goal_active = True
        fut = self.nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        fut.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Meta rechazada.')
            self._reset_goal()
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        self.get_logger().debug(f'Feedback: {feedback_msg.feedback}')

    def _get_result_callback(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Navegación terminó con estado {status}')
        else:
            self.get_logger().info('Meta alcanzada.')
        self._reset_goal()

    def _reset_goal(self):
        self.goal_active = False
        self.last_goal_end_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = RoamingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando nodo de roaming...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
