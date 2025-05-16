#!/usr/bin/env python3
"""
ROS2 Python node en Humble para filtrar mensajes de audio en /audio/raw
solo cuando hay voz o sonido significativo, republicándolos en /audio/filtered.

Se utilizan técnicas modernas:
 - WebRTC VAD para detección de voz (soporta 10/20/30 ms frames)
 - Umbral de energía RMS como respaldo

Parámetros:
 - input_topic: topic de entrada (AudioRaw)
 - output_topic: topic de salida (AudioRaw)
 - use_vad: bool, usar VAD si está disponible
 - vad_aggressiveness: nivel VAD [0-3]
 - energy_threshold: umbral RMS si no se usa VAD o VAD no soporta sample_rate
 - frame_duration_ms: duración de frame para VAD (10,20 o 30)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from sancho_msgs.msg import AudioData  # Ajusta al paquete/mensaje real

try:
    import webrtcvad

    VAD_AVAILABLE = True
except ImportError:
    VAD_AVAILABLE = False


class AudioFilterNode(Node):
    def __init__(self):
        super().__init__("audio_filter_node")
        # Parámetros
        self.declare_parameter("input_topic", "/audio/raw")
        self.declare_parameter("output_topic", "/audio/filtered")
        self.declare_parameter("use_vad", True)
        self.declare_parameter("vad_aggressiveness", 2)
        self.declare_parameter("energy_threshold", 500.0)
        self.declare_parameter("frame_duration_ms", 100)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.use_vad = self.get_parameter("use_vad").value and VAD_AVAILABLE
        self.vad = (
            webrtcvad.Vad(self.get_parameter("vad_aggressiveness").value)
            if self.use_vad
            else None
        )
        self.energy_threshold = self.get_parameter("energy_threshold").value
        self.frame_duration_ms = self.get_parameter("frame_duration_ms").value

        # Subscriber y publisher
        self.sub = self.create_subscription(
            AudioData, self.input_topic, self.callback, 10
        )
        self.pub = self.create_publisher(AudioData, self.output_topic, 10)

        self.get_logger().info(
            f"AudioFilterNode listo. VAD disponible: {VAD_AVAILABLE}, usando VAD: {self.use_vad}"
        )

    def callback(self, msg: AudioData):
        # Convertir a numpy array estéreo
        data = np.array(msg.data, dtype=np.int16)
        if msg.channels != 2:
            self.get_logger().warn("Canales != 2, reprocesando como mono")
            mono = data.astype(np.int16)
        else:
            data = data.reshape(-1, 2)
            mono = data.mean(axis=1).astype(np.int16)

        # Seleccionar método de detección
        if self.use_vad and msg.sample_rate in (8000, 16000, 32000, 48000):
            if self._has_voice(mono, msg.sample_rate):
                self.pub.publish(msg)
        else:
            if self._has_energy(mono):
                self.pub.publish(msg)

    def _has_voice(self, mono: np.ndarray, sample_rate: int) -> bool:
        # Dividir en frames de frame_duration_ms
        bytes_per_sample = 2  # int16
        frame_len = int(sample_rate * self.frame_duration_ms / 1000)
        n_frames = len(mono) // frame_len
        audio_bytes = mono.tobytes()
        for i in range(n_frames):
            start = i * frame_len * bytes_per_sample
            end = start + frame_len * bytes_per_sample
            frame = audio_bytes[start:end]
            # if frame < required length, break
            if len(frame) != frame_len * bytes_per_sample:
                break
            if self.vad.is_speech(frame, sample_rate):
                return True
        return False

    def _has_energy(self, mono: np.ndarray) -> bool:
        # RMS y comparación con umbral
        rms = np.sqrt(np.mean(mono.astype(np.float32) ** 2))
        return rms >= self.energy_threshold


def main(args=None):
    rclpy.init(args=args)
    node = AudioFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
