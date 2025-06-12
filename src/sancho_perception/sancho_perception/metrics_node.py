#!/usr/bin/env python3
"""
Nodo RelayE2EMetricsNode:
- Se suscribe a imagen original.
- Reenvía al topic de inferencia.
- Mide la latencia end-to-end (t_header_recv - t_relay_pub) y calcula su media.
"""
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class RelayE2EMetricsNode(Node):
    def __init__(self):
        super().__init__('relay_e2e_metrics_node')
        # Parámetros de topics
        self.declare_parameter('relay.input_topic', '/camera/color/image_raw')
        self.declare_parameter('relay.output_topic', '/movenet/relay_image')
        self.declare_parameter('inference.header_topic', '/movenet/detections/header')

        in_t  = self.get_parameter('relay.input_topic').value
        out_t = self.get_parameter('relay.output_topic').value
        hdr_t = self.get_parameter('inference.header_topic').value

        self.get_logger().info(f'RelayE2E: {in_t} → {out_t}')
        qos = rclpy.qos.QoSProfile(depth=10)

        # Subscripciones y publisher
        self.sub_in  = self.create_subscription(Image,  in_t,  self.on_image,  qos)
        self.pub_out = self.create_publisher(Image, out_t, qos)
        self.sub_hdr = self.create_subscription(Header, hdr_t, self.on_header, qos)

        # Métricas end-to-end
        self.total_count = 0
        self.total_sum   = 0.0

        # Mapa de tiempos de publicación por stamp
        self._timestamps = {}
        self._lock = threading.Lock()

    def on_image(self, msg: Image):
        # 1) Marcamos el instante de publicación en el header
        now_msg = self.get_clock().now().to_msg()
        msg.header.stamp = now_msg

        # 2) Publicamos inmediatamente
        self.pub_out.publish(msg)

        # 3) Guardamos el tiempo (segundos float) en el diccionario
        t_pub = now_msg.sec + now_msg.nanosec * 1e-9
        key = (now_msg.sec, now_msg.nanosec)
        with self._lock:
            self._timestamps[key] = t_pub

    def on_header(self, header: Header):
        # 1) Tiempo de llegada del header
        now = self.get_clock().now().to_msg()
        t_recv = now.sec + now.nanosec * 1e-9

        key = (header.stamp.sec, header.stamp.nanosec)
        with self._lock:
            t_pub = self._timestamps.pop(key, None)
        if t_pub is None:
            return  # si no encontramos el par, lo ignoramos

        # 2) Calcular latencia end-to-end y acumular
        latency_ms = (t_recv - t_pub) * 1000.0
        with self._lock:
            self.total_sum   += latency_ms
            self.total_count += 1
            avg_ms = self.total_sum / self.total_count

        # 3) Loguear
        self.get_logger().info(
            f'[E2E] {latency_ms:.2f} ms   (avg over {self.total_count}: {avg_ms:.2f} ms)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RelayE2EMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
