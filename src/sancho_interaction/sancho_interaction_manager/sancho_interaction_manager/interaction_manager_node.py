import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class InteractionManager(Node):
    def __init__(self):
        super().__init__('interaction_manager')

        # Crear servicios para activar/desactivar interacción completa
        self.srv_activate = self.create_service(ChangeState, 'activate_interaction', self.activate_callback)
        self.srv_deactivate = self.create_service(ChangeState, 'deactivate_interaction', self.deactivate_callback)

        # Crear clientes a los Lifecycle nodes
        self.modules = [
            self.create_client(ChangeState, 'face_detector/change_state'),
            self.create_client(ChangeState, 'audio_listener/change_state'),
            self.create_client(ChangeState, 'gesture_detector/change_state')
        ]

    async def call_lifecycle_transition(self, clients, transition_id):
        results = []
        for client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning(f'Servicio {client.srv_name} no disponible...')
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = client.call_async(req)
            await future
            results.append(future)
        return results

    async def activate_callback(self, request, response):
        self.get_logger().info('Activando módulos de interacción (Lifecycle)...')
        await self.call_lifecycle_transition(self.modules, Transition.TRANSITION_ACTIVATE)
        response.success = True
        response.message = 'Todos los módulos activados (Lifecycle)'
        return response

    async def deactivate_callback(self, request, response):
        self.get_logger().info('Desactivando módulos de interacción (Lifecycle)...')
        await self.call_lifecycle_transition(self.modules, Transition.TRANSITION_DEACTIVATE)
        response.success = True
        response.message = 'Todos los módulos desactivados (Lifecycle)'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = InteractionManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
