#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sancho_msgs.msg import AudioData

import wave
import numpy as np

class QuickChecker(Node):
    def __init__(self):
        super().__init__('quick_checker')
        self.sub = self.create_subscription(
            AudioData, '/audio/raw', self.cb, 10)

        self.duration_s = 5.0       # segundos que queremos grabar
        self.frames_needed = None   # sr * duration_s, lo calcularemos con el primer mensaje

        # Buffers de int16 para cada canal
        self.buf_ch1 = []
        self.buf_ch2 = []
        self.total_frames = 0

    def cb(self, msg: AudioData):
        # Inicializamos sólo la primera vez
        if self.frames_needed is None:
            sr = msg.sample_rate
            ch = msg.channels
            self.frames_needed = int(sr * self.duration_s)
            self.get_logger().info(f'SR={sr}, canales={ch}, necesito {self.frames_needed} frames')

        # Convertir la lista a numpy array
        data = np.array(msg.data, dtype=np.int16)
        ch = int(msg.channels)
        frames = data.size // ch
        data = data.reshape((frames, ch))

        # Acumular
        self.buf_ch1.append(data[:, 0])
        if ch > 1:
            self.buf_ch2.append(data[:, 1])
        else:
            self.buf_ch2.append(np.zeros(frames, dtype=np.int16))

        self.total_frames += frames
        self.get_logger().info(f'Acumulados {self.total_frames}/{self.frames_needed} frames')

        # ¿Tenemos ya suficiente?
        if self.total_frames >= self.frames_needed:
            self.get_logger().info('Grabación 0.5 s lista → escribiendo test_ch?.wav')
            ch1 = np.concatenate(self.buf_ch1)[:self.frames_needed]
            ch2 = np.concatenate(self.buf_ch2)[:self.frames_needed]
            self.write('test_ch1.wav', msg.sample_rate, ch1)
            self.write('test_ch2.wav', msg.sample_rate, ch2)
            self.get_logger().info('Archivos generados: test_ch1.wav, test_ch2.wav')
            rclpy.shutdown()

    def write(self, fn, sr, arr):
        with wave.open(fn, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)        # int16 = 2 bytes
            wf.setframerate(sr)
            wf.writeframes(arr.tobytes())

def main():
    rclpy.init()
    node = QuickChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
