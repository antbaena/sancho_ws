# File: tdoa_detector_node.py
import rclpy
from rclpy.node import Node
from sancho_msgs.msg import VADSegment
from std_msgs.msg import Float32
import numpy as np

# Import both methods
from .tdoa_nodes import STRATEGIES, TDOAStrategy


def compute_angle_from_delay(
    delay: float, mic_distance: float, sound_speed: float = 343.0
) -> float:
    """Convert time delay to arrival angle in degrees."""
    # sin(theta) = c * tau / d
    arg = np.clip((sound_speed * delay) / mic_distance, -1.0, 1.0)
    theta = np.degrees(np.arcsin(arg))
    return float(theta)


class SoundAngleDetector(Node):
    def __init__(self):
        super().__init__("sound_angle_detector")
        # Parameters
        self.declare_parameter("method", "gcc")  # 'gcc' or 'ncc'
        self.declare_parameter("mic_distance", 0.1225)
        self.declare_parameter("sample_rate", 48000)
        self.declare_parameter("timer_hz", 5.0)

        self.method = self.get_parameter("method").value
        self.mic_distance = self.get_parameter("mic_distance").value
        self.sample_rate = self.get_parameter("sample_rate").value
        timer_hz = self.get_parameter("timer_hz").value

        strategy_cls = STRATEGIES.get(self.method)
        if strategy_cls is None:
            raise ValueError(f"Método de TDOA desconocido: {self.method}")
        if issubclass(strategy_cls, TDOAStrategy):
            if self.method == "gcc":
                self.strategy: TDOAStrategy = strategy_cls(
                    sample_rate=self.sample_rate,
                    mic_distance=self.mic_distance,
                )
            else:
                self.strategy = strategy_cls(sample_rate=self.sample_rate)
        else:
            raise TypeError("Clase de estrategia inválida")

        # Subscriber to VAD segments
        self.sub = self.create_subscription(
            VADSegment, "/audio/vad_segment", self.vad_callback, 10
        )
        self.buffer = []  # will store incoming segments

        # Publisher for angle
        self.pub = self.create_publisher(Float32, "/audio/angle", 10)

        # Timer to process buffer periodically
        self.timer = self.create_timer(1.0 / timer_hz, self.timer_callback)
        self.get_logger().info(f"SoundAngleDetector iniciado con método={self.method}")

    def vad_callback(self, msg: VADSegment):
        # Store each segment for later processing
        self.buffer.append(msg)

    def timer_callback(self):
        if not self.buffer:
            return
        # Process all buffered segments
        for seg in self.buffer:
            left = np.array(seg.left_channel, dtype=float)
            right = np.array(seg.right_channel, dtype=float)
            tau = self.strategy.compute_delay(left, right)
            angle = compute_angle_from_delay(tau, self.mic_distance)
            # Publish result
            msg_out = Float32()
            msg_out.data = angle
            self.pub.publish(msg_out)
            self.get_logger().info(
                f"Publicado ángulo: {angle:.2f}° (método={self.method})"
            )
        # Clear buffer
        self.buffer.clear()


def main(args=None):
    rclpy.init(args=args)
    node = SoundAngleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
