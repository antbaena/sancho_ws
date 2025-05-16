#!/usr/bin/env python3
"""
ROS2 Python node que aplica GCC-PHAT para estimar el ángulo de incidencia de sonido
sobre un arreglo estéreo de dos micrófonos.

Suscribe a un topic /audio/raw con mensaje custom:
    int16[] data
    uint32 sample_rate
    uint8 channels

Publica en topic /audio/angle (std_msgs/Float32) el ángulo en grados.
"""

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float32

from sancho_msgs.msg import AudioData  # Ajusta al nombre real de tu mensaje


class GCCPHATNode(Node):
    def __init__(self):
        super().__init__("gcc_phat_node")
        # Parámetros configurables
        self.declare_parameter("input_topic", "/audio/filtered")
        self.declare_parameter("output_topic", "/audio/angle")
        self.declare_parameter("mic_distance", 0.122)  # distancia entre micrófonos (m)
        self.declare_parameter("speed_of_sound", 343.0)  # velocidad del sonido (m/s)
        self.declare_parameter(
            "filter_alpha", 0.0
        )  # coeficiente de suavizado exponencial [0-1]
        self.declare_parameter("interp", 4)  # factor de interpolación para submuestreo
        self.declare_parameter(
            "max_tau", None, ParameterDescriptor(dynamic_typing=True)
        )
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.mic_distance = self.get_parameter("mic_distance").value
        self.speed_of_sound = self.get_parameter("speed_of_sound").value
        self.max_tau = self.get_parameter("max_tau").value
        self.interp = self.get_parameter("interp").value
        self.filter_alpha = self.get_parameter("filter_alpha").value

        self.prev_angle = None

        # Subscripción y publicación
        self.sub = self.create_subscription(
            AudioData, self.input_topic, self.audio_callback, 10
        )
        self.pub = self.create_publisher(Float32, self.output_topic, 10)

        self.get_logger().info(
            f'Nodo GCC-PHAT inicializado, suscrito a "{self.input_topic}"'
        )

    def audio_callback(self, msg: AudioData):
        # Convertir datos a numpy array
        data = np.array(msg.data, dtype=np.int16)
        if msg.channels != 2:
            self.get_logger().error("El nodo solo soporta 2 canales")
            return
        data = data.reshape(-1, 2)
        sig = data[:, 0].astype(np.float32)
        refsig = data[:, 1].astype(np.float32)

        # Estimación de retardo mediante GCC-PHAT
        tau = self.gcc_phat(
            sig, refsig, fs=msg.sample_rate, max_tau=self.max_tau, interp=self.interp
        )
        # Cálculo del ángulo de incidencia
        cos_theta = (tau * self.speed_of_sound) / self.mic_distance
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        theta = np.degrees(np.arccos(cos_theta))
        # Ajustar signo según dirección del desfase
        if tau < 0:
            theta = -theta
        # Suavizado exponencial opcional
        if self.prev_angle is not None and self.filter_alpha > 0.0:
            theta = (
                self.filter_alpha * theta + (1 - self.filter_alpha) * self.prev_angle
            )
        self.prev_angle = theta

        # Publicar resultado
        msg_out = Float32()
        msg_out.data = float(theta)
        self.pub.publish(msg_out)

    @staticmethod
    def gcc_phat(sig, refsig, fs=1, max_tau=None, interp=1):
        """
        Implementación de GCC-PHAT con PHAT weighting e interpolación para resolución sub-muestral.
        retorna: tau (segundos)
        """
        # Largo para FFT
        n = sig.shape[0] + refsig.shape[0]
        nfft = 1 << (int(np.log2(n - 1)) + 1)
        # Transformadas
        SIG = np.fft.rfft(sig, n=nfft)
        REFSIG = np.fft.rfft(refsig, n=nfft)
        # Producto cruzado y PHAT weighting
        R = SIG * np.conj(REFSIG)
        R /= np.abs(R) + np.finfo(float).eps
        # Correlación inversa con interpolación
        cc = np.fft.irfft(R, n=nfft * interp)
        # Límite de desplazamiento en muestras
        max_shift = int(interp * fs * (max_tau if max_tau else (sig.shape[0] / fs)))
        if max_tau:
            max_shift = min(int(interp * fs * max_tau), cc.shape[0] // 2)
        cc = np.concatenate((cc[-max_shift:], cc[: max_shift + 1]))
        # Encuentro del pico
        shift = np.argmax(np.abs(cc)) - max_shift
        tau = shift / float(interp * fs)
        return tau


def main(args=None):
    rclpy.init(args=args)
    node = GCCPHATNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
