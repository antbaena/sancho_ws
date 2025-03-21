import rclpy
from rclpy.node import Node

from human_face_recognition_msgs.srv import AudioRecognition
from std_msgs.msg import String

from .api.gstt import transcribe

class GoogleSTT(Node):

    def __init__(self):
        super().__init__("google_stt")

        self.stt_service = self.create_service(AudioRecognition, "audio/recognition", self.audio_recognition)
        self.transcription_topic = self.create_publisher(String, "audio/transcription", 5)
        self.get_logger().info('STT inicializado correctamente')
            
    def audio_recognition(self, request, response):
        audio = request.audio
        sample_rate = request.sample_rate
        
        text = transcribe(audio, sample_rate)

        response.text = String(data=str(text))

        if text is not None:
            self.transcription_topic.publish(response.text)
        
        return response

def main(args=None):
    rclpy.init(args=args)

    google_stt = GoogleSTT()

    rclpy.spin(google_stt)
    rclpy.shutdown()