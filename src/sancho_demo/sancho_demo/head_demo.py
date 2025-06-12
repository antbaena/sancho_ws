#!/usr/bin/env python3
import random

import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PTUHumanMotion(Node):
    def __init__(self):
        super().__init__("ptu_human_motion")

        # Subscripción para obtener el estado de la cabeza (opcional)
        self.sub_joint_states = self.create_subscription(
            JointState, "/wxxms/joint_states", self.joint_state_callback, 10
        )

        # Publicador para enviar comandos de posición a la torreta
        self.pub_joint_command = self.create_publisher(
            JointGroupCommand, "/wxxms/commands/joint_group", 1
        )

        # Cliente para el servicio de modos de operación (opcional)
        self.client_operating_modes = self.create_client(
            OperatingModes, "/wxxms/set_operating_modes"
        )
        while not self.client_operating_modes.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Head service not available, waiting...")

        # Inicializar modo de operación y velocidad
        self.init_head()

        # Parámetros para movimiento natural
        self.PAN_MIN = -0.7  # rango mínimo para pan (izquierda)
        self.PAN_MAX = 0.7  # rango máximo para pan (derecha)
        self.TILT_MIN = -0.2  # rango mínimo para tilt (ligero hacia abajo)
        self.TILT_MAX = 0.2  # rango máximo para tilt (ligero hacia arriba)

        # Intervalos de tiempo aleatorios (segundos)
        self.MIN_INTERVAL = 5.0
        self.MAX_INTERVAL = 10.0

        self.timer = None
        self.schedule_next_move()

        self.get_logger().info("Nodo PTU para imitación de movimiento humano iniciado.")

    def init_head(self):
        """Configura la torreta en modo de posición con velocidad y aceleración inicializadas"""
        request = OperatingModes.Request()
        request.cmd_type = "group"
        request.name = "turret"
        request.mode = "position"
        request.profile_type = "time"
        request.profile_velocity = 2000  # Ajusta según necesites
        request.profile_acceleration = 1000  # Ajusta según necesites

        self.get_logger().info("Configurando modo de operación de la torreta...")
        future = self.client_operating_modes.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                "Modo de operación configurado con éxito. Respuesta: %s"
                % future.result()
            )
        else:
            self.get_logger().error("Error al configurar el modo de operación.")

        # Posición inicial de la torreta
        self.pub_joint_command.publish(JointGroupCommand(name="turret", cmd=[0, 0]))
        self.get_logger().info("Torreta inicializada en posición neutral.")

    def joint_state_callback(self, msg: JointState):
        # Procesamiento del estado de la cabeza
        self.get_logger().debug("Estado de la cabeza recibido.")

    def schedule_next_move(self):
        # Cancela timer previo si existe
        if self.timer:
            self.timer.cancel()
        interval = random.uniform(self.MIN_INTERVAL, self.MAX_INTERVAL)
        self.get_logger().info(
            f"Programando siguiente movimiento en {interval:.2f} segundos."
        )
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
            self.get_logger().info(
                f"Movimiento enviado: pan = {pan:.2f}, tilt = {tilt:.2f}."
            )
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


if __name__ == "__main__":
    main()
