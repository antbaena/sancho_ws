import numpy as np
from queue import Queue

import rclpy
from rclpy.node import Node

from human_face_recognition_msgs.msg import ChunkMono
from human_face_recognition_msgs.srv import AudioRecognition, HelperMode
from std_msgs.msg import String

# Poner que si solo se graba un chunk o alguna cosa asi que no se suba sobretodo si esta esperando un comando
# Poner que si no traduce nada que printee "Nothing transcribed" o algo asi
# Poner la cola con tiempo, si un mensaje tiene mas de 10s lo purga
# Pensar que hacer con la cola

class AssistantHelper(Node):

    def __init__(self, sample_rate):
        super().__init__("assistant_helper")
        self.sample_rate = sample_rate
        self.check_each_seconds = 0.5
        self.intensity_threshold = 750
        self.audio = []
        self.check_audio = []
        self.previous_chunk = []
        self.no_more_audio = -1
        self.listen_for_name()

        self.text_publisher = self.create_publisher(String, 'audio/assistant/text', 10)
        self.mode_service = self.create_service(HelperMode, "audio/helper/mode", self.helper_mode)
        
        self.audio_recognition_client = self.create_client(AudioRecognition, 'audio/recognition')
        while not self.audio_recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Audio recognition service not available, waiting again...')
            
        self.transcribe_queue = Queue(maxsize=1)
        self.subscription = self.create_subscription(ChunkMono, 'AudioRaw/microphone/mono', self.microphone_callback, 10)

    def helper_mode(self, request, response):
        if request.mode:
            self.listen_for_command()
        else:
            self.listen_for_name()
        
        response.result = 1
        return response

    def listen_for_command(self):
        self.record_seconds = -1
    
    def listen_for_name(self):
        self.record_seconds = 2

    def audio_average_intensity(self, audio):
        average_intensity = np.mean(np.abs(audio))
        if average_intensity < 0:
            average_intensity = 32767
            
        return average_intensity

    def microphone_callback(self, msg):
        new_audio = list([np.int16(x) for x in msg.chunk_mono])
        self.check_audio = self.check_audio + new_audio

        if len(self.check_audio) >= self.check_each_seconds * self.sample_rate:
            avg_intensity = self.audio_average_intensity(self.check_audio)
            print(str(self.check_each_seconds) + " seconds: " + str(avg_intensity))

            if avg_intensity >= self.intensity_threshold:
                self.no_more_audio = 0

                if len(self.audio) == 0:
                    self.audio = self.audio + self.previous_chunk # En caso de sentir intensidad, metemos
                    # el chunk previo para que no se parta el nombre en dos

                self.audio = self.audio + self.check_audio
                print("Chunk attached (" + str(len(self.audio) / self.sample_rate) + " seconds)")
            elif self.no_more_audio >= 0:
                self.audio = self.audio + self.check_audio # Para que no se corte el audio
                # la idea es que cuando haya un trozo de audio, adjuntar el chunk anterior y el posterior asi seguro que coge todo
                self.no_more_audio = 1
                print("No more audio detected...")
            else:
                print("No audio detected yet")

            self.previous_chunk = self.check_audio
            self.check_audio = []

        # Si esta en modo escuchar por comando y no hay mas chunks o si esta en modo escuchar nombre y se supera el tiempo
        # Poner que si te pasas de los segundos minimos igualmente pille el chunk posterior para la traduccion
        # Al final del codigo poner if el or de la derecha entonces no more audio = 1 y se hace solo
        if self.no_more_audio == 1 or (self.record_seconds > 0 and len(self.audio) >= self.record_seconds * self.sample_rate):
            if self.transcribe_queue.qsize() < 1:
                self.transcribe_queue.put(self.audio)
            else:
                self.get_logger().info("Transcribe Queue IS FULL!!!")
            
            self.audio = []
            self.check_audio = []
            self.no_more_audio = -1

    def audio_recognition_request(self, audio):
        audio_recognition_request = AudioRecognition.Request()

        audio = np.ndarray.tolist(np.array(audio)) # This is weird. Without this conversion it doesn't work. Need to see why

        audio_recognition_request.audio = audio
        audio_recognition_request.sample_rate = self.sample_rate

        future_audio_recognition = self.audio_recognition_client.call_async(audio_recognition_request)
        rclpy.spin_until_future_complete(self, future_audio_recognition)
        result_audio_recognition = future_audio_recognition.result()

        return result_audio_recognition.text

def main(args=None):
    rclpy.init(args=args)

    assistant_helper = AssistantHelper(sample_rate=48000)

    while rclpy.ok():
        if not assistant_helper.transcribe_queue.empty():
            audio = assistant_helper.transcribe_queue.get()
            print("Transcribing...")
       
            rec_msg = assistant_helper.audio_recognition_request(audio)
            rec = None if rec_msg.data == "None" else rec_msg.data
            
            if rec is not None:
                print("Sending to assistant " + str(len(audio) / assistant_helper.sample_rate) + " seconds")
                assistant_helper.text_publisher.publish(String(data=rec))
            else:
                print("Transcription result is None")

            print("Average intensity: " + str(assistant_helper.audio_average_intensity(audio)))

        rclpy.spin_once(assistant_helper)

    rclpy.shutdown()