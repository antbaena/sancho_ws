import rclpy
from rclpy.node import Node

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        self.get_logger().info('Orquestador iniciado!')

        # Aquí pondremos:
        # - Subscripción a /group_detection
        # - Clients para LifeCycle (activar/desactivar nodos)
        # - Action Client para nav2
        # - Publisher para interacción social
        # - FSM de estados

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
