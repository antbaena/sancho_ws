#!/usr/bin/env python3
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from sancho_msgs.msg import AudioData


class MicrophoneNode(Node):
    def __init__(self):
        super().__init__("microphone_capturer_node")

        # 1) Declaración de parámetros (puedes sobreescribirlos con ros2 param set)
        self.declare_parameter("device_search_name", "ORBBEC")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("channels", 2)
        self.declare_parameter("topic_name", "/audio/raw")
        self.declare_parameter("frame_id", "microphone_frame")

        # 2) Lectura de parámetros
        name_part = self.get_parameter("device_search_name").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.chunk_size = self.get_parameter("chunk_size").value
        self.channels = self.get_parameter("channels").value
        topic = self.get_parameter("topic_name").value
        self.frame_id = self.get_parameter("frame_id").value

        # 3) Publisher
        self.pub_audio = self.create_publisher(AudioData, topic, 10)

        # 4) Inicializar PyAudio y encontrar dispositivo
        self.pa = pyaudio.PyAudio()
        self.device_index = self._find_device_index(name_part)
        if self.device_index is None:
            self.get_logger().error(f'Dispositivo "{name_part}" no encontrado.')
            raise RuntimeError("Micrófono no encontrado.")

        # 5) Abrir stream (excepción_on_overflow=False para robustez)
        self.stream = self.pa.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=self.device_index,
        )
        self.get_logger().info(
            f'Stream abierto en "{name_part}" (index={self.device_index}), '
            f"{self.channels} canales @ {self.sample_rate} Hz."
        )

        # 6) Timer para leer datos periódicamente
        periodo = float(self.chunk_size) / self.sample_rate
        self.create_timer(periodo, self._timer_callback)

    def _find_device_index(self, name_part: str) -> int:
        """Busca el primer dispositivo cuyo nombre contenga name_part (case-insensitive)."""
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            if name_part.lower() in info.get("name", "").lower():
                self.get_logger().info(
                    f'Dispositivo encontrado: "{info["name"]}" (index={i})'
                )
                return i
        return None

    def _timer_callback(self):
        """Lee un chunk de audio y lo publica."""
        try:
            datos = self.stream.read(self.chunk_size, exception_on_overflow=False)
            # Crear y rellenar la cabecera
            hdr = Header()
            hdr.stamp = self.get_clock().now().to_msg()
            hdr.frame_id = self.frame_id

            # Construir mensaje con cabecera
            msg = AudioData(
                header=hdr,
                data=datos,
                sample_rate=self.sample_rate,
                channels=self.channels,
            )
            self.pub_audio.publish(msg)
        except IOError as e:
            # solo catch de IOErrors (overflow, dispositivo desconectado, etc.)
            self.get_logger().warn(f"Overflow o I/O error: {e}. Reabriendo stream…")
            self._reopen_stream()
        except Exception as e:
            self.get_logger().error(f"Error inesperado al leer audio: {e}")
            self._reopen_stream()

    def _reopen_stream(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
        except Exception:
            pass
        self.stream = self.pa.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=self.device_index,
        )

    def destroy_node(self):
        """Cierre limpio del stream y PyAudio."""
        try:
            self.stream.stop_stream()
            self.stream.close()
        except Exception:
            pass
        self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
