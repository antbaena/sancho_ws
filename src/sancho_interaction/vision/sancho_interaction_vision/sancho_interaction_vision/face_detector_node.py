#!/usr/bin/env python3

import cv2
import rclpy
import time
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Image
from human_face_recognition_msgs.srv import Detection
from .hri_bridge import HRIBridge

class HumanFaceDetector(LifecycleNode):

    def __init__(self):
        """Inicializa el nodo detector de caras."""
        super().__init__("human_face_detector")

        # Declarar parámetros
        show_metrics_param = self.declare_parameter("show_metrics", False)
        active_cnn_param = self.declare_parameter("active_cnn", False)

        self.show_metrics = show_metrics_param.get_parameter_value().bool_value
        self.active_cnn = active_cnn_param.get_parameter_value().bool_value

        self.get_faces = None
        self.br = HRIBridge()

        # Variables para los recursos ROS
        self.detection_service = None
        self.detection_publisher = None

        self.get_logger().info("Nodo HumanFaceDetector inicializado.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configura el nodo: importa el detector correcto y prepara recursos."""
        self.get_logger().info("Configurando nodo de detección de caras...")

        try:
            if self.active_cnn:
                from .detectors.detector_dlib_cnn import get_faces
                self.get_logger().info("Importando detector DLIB CNN")
            else:
                from .detectors.detector_dlib_frontal import get_faces
                self.get_logger().info("Importando detector DLIB Frontal")

            self.get_faces = get_faces

        except Exception as e:
            self.get_logger().error(f"Error importando detector: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activa el servicio y el publicador."""
        self.get_logger().info("Activando nodo...")

        try:
            self.detection_service = self.create_service(
                Detection, "detection", self.detection_callback
            )
            self.detection_publisher = self.create_publisher(
                Image, "camera/color/detection", 10
            )
        except Exception as e:
            self.get_logger().error(f"Error creando interfaces ROS: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Desactiva el servicio y el publicador."""
        self.get_logger().info("Desactivando nodo...")

        if self.detection_service is not None:
            self.destroy_service(self.detection_service)
            self.detection_service = None

        if self.detection_publisher is not None:
            self.destroy_publisher(self.detection_publisher)
            self.detection_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Limpia el estado interno."""
        self.get_logger().info("Limpiando recursos internos...")

        self.get_faces = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Apaga el nodo de forma limpia."""
        self.get_logger().info("Apagando nodo de detección de caras...")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Manejo de errores del nodo."""
        self.get_logger().error("Error en el nodo de detección de caras.")
        return TransitionCallbackReturn.SUCCESS

    def detection_callback(self, request, response):
        """Servicio de detección."""
        start_detection = time.time()

        try:
            frame = self.br.imgmsg_to_cv2(request.frame, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Mejora de imagen
            image_bilateral = cv2.bilateralFilter(gray, d=5, sigmaColor=50, sigmaSpace=50)
            gray_equalized = cv2.equalizeHist(image_bilateral)

            frame_ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
            frame_ycrcb[:, :, 0] = gray_equalized
            frame_equalized = cv2.cvtColor(frame_ycrcb, cv2.COLOR_YCrCb2BGR)

            postProcess_image = frame_equalized

            if self.get_faces:
                positions, scores, _ = self.get_faces(postProcess_image)
                positions_msg, scores_msg = self.br.detector_to_msg(positions, scores)
                self.get_logger().info(f"Caras detectadas: {len(positions_msg)}")

                response.positions = positions_msg
                response.scores = scores_msg

                for x, y, w, h in positions:
                    cv2.rectangle(postProcess_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                detection_imgmsg = self.br.cv2_to_imgmsg(postProcess_image, "bgr8")
                if self.detection_publisher is not None:
                    self.detection_publisher.publish(detection_imgmsg)

            else:
                self.get_logger().warn("Detector no inicializado.")

            detection_time = time.time() - start_detection
            response.detection_time = detection_time
            if self.show_metrics:
                self.get_logger().info(f"Tiempo de detección: {detection_time:.3f}s")

        except Exception as e:
            self.get_logger().error(f"Error durante la detección: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    human_face_detector = HumanFaceDetector()

    try:
        rclpy.spin(human_face_detector)
    except KeyboardInterrupt:
        human_face_detector.get_logger().info("Interrupción por teclado, cerrando nodo...")
    finally:
        human_face_detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
