#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from lifecycle_msgs.srv import ChangeState

# Definición de estados para la FSM del orquestador
class OrchestratorState:
    BUSCANDO = 'BUSCANDO'
    NAVEGANDO = 'NAVEGANDO'
    EVALUANDO = 'EVALUANDO'
    SOCIALIZANDO = 'SOCIALIZANDO'

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        self.set_state(OrchestratorState.BUSCANDO)
        self.get_logger().info(f"Orquestador iniciado en estado: {self.state}")

        # Parámetros de timeout (segundos) para evitar deadlocks en cada estado
        self.navigation_timeout = 20.0   # Máximo tiempo permitido en NAVEGANDO
        self.evaluation_timeout = 5.0    # Máximo tiempo permitido en EVALUANDO
        self.social_timeout = 30.0       # Máximo tiempo permitido en SOCIALIZANDO

        # Subscripción al topic de waypoints generados por el nodo de detección (ej. '/group_waypoint')
        self.create_subscription(PoseStamped, '/group_waypoint', self.goal_callback, 10)

        # Cliente de acción para navegación (Nav2)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher para activar/desactivar la interacción (nodo de interacción)
        self.interaction_pub = self.create_publisher(Bool, '/interaction_control', 10)

        # Timer para evaluar periódicamente el tiempo en cada estado
        self.create_timer(0.5, self.timer_callback)

    def set_state(self, new_state: str):
        """Cambia el estado actual y reinicia el temporizador interno."""
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"Estado cambiado a: {new_state}")

    def goal_callback(self, msg: PoseStamped):
        """Recibe el waypoint del nodo de detección de grupos."""
        if self.state != OrchestratorState.BUSCANDO:
            self.get_logger().info("Waypoint recibido, pero el sistema ya está ocupado.")
            return

        self.get_logger().info("Waypoint recibido. Pasando a estado NAVEGANDO.")
        # Antes de navegar, desactivamos la detección para evitar interferencias
        self.deactivate_group_detection()
        self.set_state(OrchestratorState.NAVEGANDO)
        self.send_navigation_goal(msg)

    def send_navigation_goal(self, goal_msg: PoseStamped):
        """Envía el goal de navegación al servidor de acciones de Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de navegación no disponible.")
            self.set_state(OrchestratorState.BUSCANDO)
            self.activate_group_detection()
            return

        goal_req = NavigateToPose.Goal()
        goal_req.pose = goal_msg

        self.nav_client.send_goal_async(goal_req, feedback_callback=self.feedback_callback)\
            .add_done_callback(self.navigation_result_callback)

    def feedback_callback(self, feedback_msg):
        """Procesa feedback de navegación (puedes ampliarlo según necesites)."""
        # Por ejemplo: self.get_logger().info("Feedback recibido de navegación")
        pass

    def navigation_result_callback(self, future):
        """Callback cuando termina la navegación."""
        try:
            result = future.result().result
            self.get_logger().info("Navegación completada, pasando a evaluación social.")
            self.set_state(OrchestratorState.EVALUANDO)
            self.evaluate_social_context()
        except Exception as e:
            self.get_logger().error(f"Error en navegación: {e}")
            self.set_state(OrchestratorState.BUSCANDO)
            self.activate_group_detection()

    def evaluate_social_context(self):
        """Evalúa si las condiciones son favorables para socializar.
           Aquí puedes integrar lógica basada en sensores adicionales."""
        social_possible = True  # Suponemos condiciones favorables por defecto

        if social_possible:
            self.get_logger().info("Condiciones favorables para socializar. Activando interacción.")
            self.activate_social_interaction()
        else:
            self.get_logger().info("No es posible socializar en este momento. Reiniciando búsqueda.")
            self.set_state(OrchestratorState.BUSCANDO)
            self.activate_group_detection()

    def activate_social_interaction(self):
        """Activa el nodo de interacción vía Lifecycle y publica el comando para iniciar interacción."""
        if self.change_lifecycle_state("social_interaction_node", transition_id=3):
            self.interaction_pub.publish(Bool(data=True))
            self.set_state(OrchestratorState.SOCIALIZANDO)
            self.get_logger().info("Interacción social activada.")
        else:
            self.get_logger().error("No se pudo activar el nodo de interacción social.")
            self.set_state(OrchestratorState.BUSCANDO)
            self.activate_group_detection()

    def finish_social_interaction(self):
        """Finaliza la interacción, desactiva el nodo de interacción y reactiva la detección de grupos."""
        self.get_logger().info("Finalizando interacción social.")
        self.change_lifecycle_state("social_interaction_node", transition_id=4)
        self.interaction_pub.publish(Bool(data=False))
        self.activate_group_detection()
        self.set_state(OrchestratorState.BUSCANDO)

    def timer_callback(self):
        """Controla los timeouts de cada estado para evitar deadlocks."""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9

        if self.state == OrchestratorState.NAVEGANDO and elapsed > self.navigation_timeout:
            self.get_logger().warn("Timeout en NAVEGANDO. Reiniciando búsqueda.")
            self.set_state(OrchestratorState.BUSCANDO)
            self.activate_group_detection()
        elif self.state == OrchestratorState.EVALUANDO and elapsed > self.evaluation_timeout:
            self.get_logger().warn("Timeout en EVALUANDO. Reiniciando búsqueda.")
            self.set_state(OrchestratorState.BUSCANDO)
            self.activate_group_detection()
        elif self.state == OrchestratorState.SOCIALIZANDO and elapsed > self.social_timeout:
            self.get_logger().warn("Timeout en SOCIALIZANDO. Finalizando interacción.")
            self.finish_social_interaction()

    def change_lifecycle_state(self, node_name: str, transition_id: int) -> bool:
        """
        Cambia el estado de un nodo Lifecycle llamando a su servicio /<node_name>/change_state.
        transition_id: 3 para activar, 4 para desactivar (según IDs estándar de lifecycle_msgs).
        """
        service_name = f'/{node_name}/change_state'
        client = self.create_client(ChangeState, service_name)

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Servicio {service_name} no disponible.")
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Transición {transition_id} en {node_name} exitosa.")
            return True
        else:
            self.get_logger().error(f"Fallo en la transición {transition_id} de {node_name}.")
            return False

    def deactivate_group_detection(self) -> bool:
        """Desactiva el nodo de detección de grupos."""
        if self.change_lifecycle_state("group_detection_node", transition_id=4):
            self.get_logger().info("Nodo de detección de grupos desactivado.")
            return True
        else:
            self.get_logger().error("No se pudo desactivar el nodo de detección de grupos.")
            return False

    def activate_group_detection(self) -> bool:
        """Activa el nodo de detección de grupos."""
        if self.change_lifecycle_state("group_detection_node", transition_id=3):
            self.get_logger().info("Nodo de detección de grupos activado.")
            return True
        else:
            self.get_logger().error("No se pudo activar el nodo de detección de grupos.")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando orquestador.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
