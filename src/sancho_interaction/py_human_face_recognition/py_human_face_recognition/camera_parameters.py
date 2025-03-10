import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile
import numpy as np
from human_face_recognition_msgs.srv import GetCameraParameters  # Asegúrate de importar tu servicio

class CameraParametersNode(Node):
    def __init__(self):
        super().__init__('camera_parameters_node')
        
        # Inicializar variables para almacenar los parámetros
        self.camera_matrix = None
        self.camera_matrix_inv = None
        self.dist_coeffs = None

        # Crear el suscriptor al tópico de CameraInfo
        qos_profile = QoSProfile(depth=10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, qos_profile)
        
        # Crear un temporizador que actualice los parámetros cada minuto
        self.timer = self.create_timer(60.0, self.update_parameters)

        # Inicialmente no se crea el servicio
        self.camera_parameters_service = None

    def camera_info_callback(self, msg):
        # Extraer la matriz de la cámara y los coeficientes de distorsión del mensaje
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)  # Calcula la matriz inversa
        self.dist_coeffs = np.array(msg.d)

        # Verifica si los parámetros han sido inicializados y crea el servicio
        if self.camera_parameters_service is None:
            self.camera_parameters_service = self.create_service(GetCameraParameters, 'get_camera_parameters', self.get_camera_parameters_callback)
            self.get_logger().info('Camera parameters service is now available.')

    def update_parameters(self):
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            self.get_logger().info('Camera parameters updated.')
        else:
            self.get_logger().warning('Camera parameters not available yet.')

    def get_camera_parameters_callback(self, request, response):
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            response.camera_matrix = self.camera_matrix.flatten().tolist()
            response.camera_matrix_inv = self.camera_matrix_inv.flatten().tolist()  # Matriz inversa
            response.dist_coeffs = self.dist_coeffs.tolist()
            self.get_logger().info('Camera parameters sent in response.')
        else:
            response.camera_matrix = []
            response.camera_matrix_inv = []
            response.dist_coeffs = []
            self.get_logger().warning('Camera parameters not available yet.')
        return response


def main(args=None):
    rclpy.init(args=args)
    camera_parameters_node = CameraParametersNode()
    rclpy.spin(camera_parameters_node)

    # Shutdown
    camera_parameters_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
