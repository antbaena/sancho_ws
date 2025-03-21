import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from human_face_recognition_msgs.srv import PlayAudio
import sounddevice as sd
import numpy as np
import threading

class SpeakerController(Node):
    def __init__(self):
        super().__init__('speaker_controller')
        # Crear el servicio para reproducir audio
        self.srv_play = self.create_service(PlayAudio, 'play_audio', self.play_audio_callback)
        # Crear el servicio para detener audio
        self.srv_stop = self.create_service(Trigger, 'stop_audio', self.stop_audio_callback)

        self.is_playing = False
        self.audio_thread = None
        self.stop_event = threading.Event()

        self.get_logger().info("Speaker Controller inicializado correctamente.")

    def play_audio_callback(self, request, response):
        self.get_logger().info("Play Audio recibido")
        if self.is_playing:
            response.success = False
            response.message = "Audio is already playing."
            return response

        # Convertir los datos recibidos a un array de numpy
        audio_data = np.array(request.audio_data, dtype=np.float64)

        # Iniciar el hilo para reproducir el audio
        self.audio_thread = threading.Thread(target=self.play_audio, args=(audio_data,))
        self.audio_thread.start()
        
        response.success = True
        response.message = "Playing audio data."
        return response

    def play_audio(self, audio_data):
        self.is_playing = True
        self.stop_event.clear()  # Asegurarse de que el evento de parada esté limpio

        try:
            # Reproducir el audio recibido
            sd.play(audio_data, samplerate=44100)
            sd.wait()  # Esperar hasta que la reproducción termine o se detenga
        except Exception as e:
            self.get_logger().error(f"Error al reproducir el audio: {e}")
        finally:
            self.is_playing = False

    def stop_audio_callback(self, request, response):
        self.get_logger().info("Parando audio...")
        # Detener la reproducción de audio
        sd.stop()
        self.stop_event.set()  # Indicar que se ha solicitado detener

        # Asegurarse de que el hilo de reproducción ha terminado
        if self.audio_thread is not None:
            self.audio_thread.join()

        self.is_playing = False
        self.get_logger().info("Audio detenido con éxito")
        response.success = True
        response.message = "Audio playback stopped."
        return response

def main(args=None):
    rclpy.init(args=args)
    speaker_controller = SpeakerController()
    rclpy.spin(speaker_controller)
    speaker_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
