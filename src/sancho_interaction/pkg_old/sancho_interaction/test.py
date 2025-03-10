#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes

class PTUCentrado(Node):
    def __init__(self):
        super().__init__('ptu_centrado')
        
        # Suscripción para obtener los estados de la cabeza
        self.sub_detection_data = self.create_subscription(
            JointState,
            "/wxxms/joint_states",
            self.head_position_callback,
            10
        )
        
        # Publicador para enviar comandos de posición
        self.pub_neck_joint = self.create_publisher(
            JointGroupCommand,
            "/wxxms/commands/joint_group",
            1
        )
        
        # Cliente para el servicio de modos de operación
        self.serv_neck_op = self.create_client(
            OperatingModes,
            "/wxxms/set_operating_modes"
        )
        
        # Espera a que el servicio esté disponible
        while not self.serv_neck_op.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Head service not available, waiting...")
        
        self.get_logger().info("Cabeza inicializada correctamente.")
        
        # (Opcional) Aquí podrías llamar al servicio para establecer el modo de operación.
        # Por ejemplo:
        # req = OperatingModes.Request()
        # req.operating_modes = [0, 0]  # Ajusta según la definición del servicio
        # future = self.serv_neck_op.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # self.get_logger().info("Modo de operación configurado.")
        
        # Crea un timer para enviar el comando de centrado
        self.timer = self.create_timer(1.0, self.send_command)

    def head_position_callback(self, msg: JointState):
        # Callback para procesar datos de estado (puedes ampliar según tus necesidades)
        self.get_logger().debug("Recibido estado de la cabeza.")

    def send_command(self):
        # Prepara el mensaje de comando para centrar la PTU en [0, 0]
        msg = JointGroupCommand()
        # Se asume que el primer elemento es para 'pan' y el segundo para 'tilt'
        msg.cmd = [0.0, 0.0]
        self.pub_neck_joint.publish(msg)
        self.get_logger().info("Comando enviado: PTU centrado en [0, 0].")
        # Se cancela el timer si solo se requiere enviar el comando una vez
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    nodo = PTUCentrado()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
