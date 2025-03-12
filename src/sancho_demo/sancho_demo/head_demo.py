#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes
import random

class PTUHumanMotion(Node):
    def __init__(self):
        super().__init__('ptu_human_motion')
        
        # Subscripción para obtener el estado de la cabeza (opcional)
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/wxxms/joint_states",
            self.joint_state_callback,
            10
        )
        
        # Publicador para enviar comandos de posición a la torreta
        self.pub_joint_command = self.create_publisher(
            JointGroupCommand,
            "/wxxms/commands/joint_group",
            1
        )
        
        # Cliente para el servicio de modos de operación (opcional)
        self.client_operating_modes = self.create_client(
            OperatingModes,
            "/wxxms/set_operating_modes"
        )
        while not self.client_operating_modes.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Head service not available, waiting...")
                
        # Parámetros para movimiento natural
        self.PAN_MIN = -0.5   # rango mínimo para pan (izquierda)
        self.PAN_MAX = 0.5    # rango máximo para pan (derecha)
        self.TILT_MIN = -0.2  # rango mínimo para tilt (ligero hacia abajo)
        self.TILT_MAX = 0.2   # rango máximo para tilt (ligero hacia arriba)
        
        # Intervalos de tiempo aleatorios (segundos)
        self.MIN_INTERVAL = 2.0
        self.MAX_INTERVAL = 5.0
        
        self.timer = None
        self.schedule_next_move()
        
        self.get_logger().info("Nodo PTU para imitación de movimiento humano iniciado.")

    def joint_state_callback(self, msg: JointState):
        # Procesamiento del estado de la cabeza (puedes ampliarlo según tus necesidades)
        self.get_logger().debug("Estado de la cabeza recibido.")

    def schedule_next_move(self):
        # Cancela timer previo si existe
        if self.timer:
            self.timer.cancel()
        interval = random.uniform(self.MIN_INTERVAL, self.MAX_INTERVAL)
        self.get_logger().info(f"Programando siguiente movimiento en {interval:.2f} segundos.")
        # Creamos un timer que simula una ejecución única
        self.timer = self.create_timer(interval, self.timer_callback)

    def timer_callback(self):
        # Cancelamos el timer actual para que no se repita automáticamente
        if self.timer:
            self.timer.cancel()
        
        try:
            # Se generan valores aleatorios dentro de los rangos definidos
            pan = random.uniform(self.PAN_MIN, self.PAN_MAX)
            tilt = random.uniform(self.TILT_MIN, self.TILT_MAX)
            
            # Preparamos y enviamos el mensaje de comando
            msg = JointGroupCommand()
            msg.name = "turret"  # Se especifica el grupo de articulaciones
            msg.cmd = [pan, tilt]
            self.pub_joint_command.publish(msg)
            self.get_logger().info(f"Movimiento enviado: pan = {pan:.2f}, tilt = {tilt:.2f}.")
        except Exception as e:
            self.get_logger().error(f"Error al enviar comando: {e}")
        
        # Programa el siguiente movimiento
        self.schedule_next_move()

def main(args=None):
    rclpy.init(args=args)
    node = PTUHumanMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción del teclado, cerrando nodo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
