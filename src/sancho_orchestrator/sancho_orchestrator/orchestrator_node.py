#!/usr/bin/env python3
"""
Orchestrator Node for ROS2 Humble
  - FSM states: BUSCANDO, NAVEGANDO, EVALUANDO, SOCIALIZANDO
  - Uses Nav2 ActionClient to navigate, lifecycle services to manage nodes,
    and parameters for easy tuning.
  - Fully asynchronous _change_node_state without blocking, ensuring
    sequence: e.g. group detection off completes before sending goal.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.msg import Transition
from action_msgs.msg import GoalStatus


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = OrchestratorNode(executor)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Orchestrator Node interrumpido.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


class OrchestratorState:
    BUSCANDO = 'BUSCANDO'
    NAVEGANDO = 'NAVEGANDO'
    EVALUANDO = 'EVALUANDO'
    SOCIALIZANDO = 'SOCIALIZANDO'


class OrchestratorNode(Node):
    def __init__(self, executor):
        super().__init__('orchestrator_node')
        self.executor = executor
        # Declare parameters
        self.declare_parameter('navigation_timeout', 20.0)
        self.declare_parameter('evaluation_timeout', 5.0)
        self.declare_parameter('social_timeout', 30.0)
        self.declare_parameter('group_waypoint_topic', '/group_waypoint')
        self.declare_parameter('navigate_action_name', 'navigate_to_pose')
        self.declare_parameter('social_node_name', 'interaction_manager')
        self.declare_parameter('group_node_name', 'group_waypoint_generator_lifecycle')

        # Load params
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.evaluation_timeout = self.get_parameter('evaluation_timeout').value
        self.social_timeout = self.get_parameter('social_timeout').value
        self.waypoint_topic = self.get_parameter('group_waypoint_topic').value
        self.navigate_action = self.get_parameter('navigate_action_name').value
        self.social_node = self.get_parameter('social_node_name').value
        self.group_node = self.get_parameter('group_node_name').value

        # Initial state
        self.current_state = OrchestratorState.BUSCANDO
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"Iniciado en estado: {self.current_state}")

        # Setup subscriptions, clients
        qos = QoSProfile(depth=10)
        self.waypoint_sub = self.create_subscription(
            PoseStamped, self.waypoint_topic, self.goal_callback, qos
        )
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)

        # Activate group detection on startup
        self._change_node_state(
            self.group_node,
            Transition.TRANSITION_ACTIVATE,
            on_done=lambda ok: self.get_logger().info("Detección de grupos activada")
        )

        self.timer = self.create_timer(0.5, self._on_timer)

    def _set_state(self, new_state):
        self.current_state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"Estado cambiado a: {new_state}")

    def goal_callback(self, msg: PoseStamped):
        if self.current_state != OrchestratorState.BUSCANDO:
            return
        self.get_logger().info("Waypoint recibido. Desactivando detección de grupos...")
        # First, deactivate group detection, then navigate
        self._change_node_state(
            self.group_node,
            Transition.TRANSITION_DEACTIVATE,
            on_done=lambda success: self._after_group_off(success, msg)
        )

    def _after_group_off(self, success, msg: PoseStamped):
        if not success:
            self.get_logger().error("Falló desactivar detección de grupos. Volviendo a BUSCANDO.")
            return self._recover_to_search()
        self.get_logger().info("Group detection off")
        # Now safe to navigate
        self._set_state(OrchestratorState.NAVEGANDO)
        self._send_navigation_goal(msg)

    def _send_navigation_goal(self, goal: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de navegación no disponible.")
            return self._recover_to_search()
        self.get_logger().info("Enviando objetivo de navegación.")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        fut = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        fut.add_done_callback(self._on_goal_response)

    def _feedback_callback(self, feedback_msg):
        self.get_logger().debug(
            f"Feedback: pose actual = {feedback_msg.feedback.current_pose}"
        )

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rechazado.")
            return self._recover_to_search()
        self.get_logger().info("Goal aceptado, esperando resultado…")
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navegación exitosa.")
            self._set_state(OrchestratorState.EVALUANDO)
            # Activate social after evaluation
            self._change_node_state(
                self.social_node,
                Transition.TRANSITION_ACTIVATE,
                on_done=self._after_activate_social
            )
        else:
            self.get_logger().warn("Navegación fallida.")
            self._recover_to_search()

    def _after_activate_social(self, success):
        if success:
            self.get_logger().info("Social node activated.")
            self._set_state(OrchestratorState.SOCIALIZANDO)
        else:
            self.get_logger().error("No pude activar socialNode.")
            self._recover_to_search()

    def _on_timer(self):
        self.get_logger().info(f"Estado actual: {self.current_state}")
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if self.current_state == OrchestratorState.NAVEGANDO and elapsed > self.navigation_timeout:
            self.get_logger().warn("Timeout NAVEGANDO.")
            self._recover_to_search()
        elif self.current_state == OrchestratorState.EVALUANDO and elapsed > self.evaluation_timeout:
            self.get_logger().warn("Timeout EVALUANDO.")
            self._recover_to_search()
        elif self.current_state == OrchestratorState.SOCIALIZANDO and elapsed > self.social_timeout:
            self.get_logger().warn("Timeout SOCIALIZANDO.")
            self._change_node_state(
                self.social_node,
                Transition.TRANSITION_DEACTIVATE,
                on_done=lambda ok: self.get_logger().info("Social off"),
                on_done_chain=self._recover_to_search
            )

    def _change_node_state(self, node_name, transition_id, on_done=None, on_done_chain=None):
        # Async GetState
        get_cli = self.create_client(GetState, f'/{node_name}/get_state')
        if not get_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{node_name}/get_state no disponible")
            if on_done: on_done(False)
            return
        fut = get_cli.call_async(GetState.Request())
        fut.add_done_callback(
            lambda f: self._on_get_state(f, node_name, transition_id, on_done, on_done_chain)
        )

    def _on_get_state(self, future, node_name, transition_id, on_done, on_done_chain):
        res = future.result()
        if res is None:
            self.get_logger().error(f"Error get_state de {node_name}")
            if on_done: on_done(False)
            return
        cur = res.current_state.id
        # Already desired?
        if ((transition_id == Transition.TRANSITION_ACTIVATE and cur == LifecycleState.PRIMARY_STATE_ACTIVE) or
            (transition_id == Transition.TRANSITION_DEACTIVATE and cur == LifecycleState.PRIMARY_STATE_INACTIVE)):
            if on_done: on_done(True)
            if on_done_chain: on_done_chain()
            return
        # Async ChangeState
        cli = self.create_client(ChangeState, f'/{node_name}/change_state')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{node_name}/change_state no disponible")
            if on_done: on_done(False)
            return
        req = ChangeState.Request()
        req.transition.id = transition_id
        ch_fut = cli.call_async(req)
        ch_fut.add_done_callback(
            lambda f: self._on_change_change_state(f, node_name, on_done, on_done_chain)
        )

    def _on_change_change_state(self, future, node_name, on_done, on_done_chain):
        res = future.result()
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f"Transición fallida en {node_name}")
        if on_done: on_done(ok)
        if ok and on_done_chain:
            on_done_chain()

    def _recover_to_search(self):
        # Reactivate group and go back to BUSCANDO
        self._change_node_state(
            self.group_node,
            Transition.TRANSITION_ACTIVATE,
            on_done=lambda ok: self._set_state(OrchestratorState.BUSCANDO)
        )


if __name__ == '__main__':
    main()
