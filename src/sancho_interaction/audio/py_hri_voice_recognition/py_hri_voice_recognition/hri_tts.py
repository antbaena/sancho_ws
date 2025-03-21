import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from .melo_tts import get_audio_file

from human_face_recognition_msgs.srv import PlayAudio

class TTSNode(Node):
    
    def __init__(self):
        super().__init__("hri_tts")
        
        self.text = None
        self.tts_sub = self.create_subscription(String, "input_tts", self.tts_callback, 10)
        self.audio_play_client = self.create_client(PlayAudio, "play_audio")
        while not self.audio_play_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando por el servicio...')
        
        self.get_logger().info("HRI TTS inicializado correctamente.")
        
    def tts_callback(self, msg):
        self.text = msg.data
        if self.text is None:
            self.get_logger().info("Mensaje None recibido. Ignorando...")

class TTSlogic:
    def __init__(self):
        self.node = TTSNode()
        
    def play_audio_request(self, audio_data):
        audio_data = list(map(float, audio_data))
        
        play_audio_request = PlayAudio.Request()
        play_audio_request.audio_data = audio_data

        future_play_audio = self.node.audio_play_client.call_async(play_audio_request)
        rclpy.spin_until_future_complete(self.node, future_play_audio)
        result_play_audio = future_play_audio.result()

        return result_play_audio.success, result_play_audio.message
        
    def spin(self):
        while rclpy.ok():
            if self.node.text is not None:
                text = self.node.text
                self.node.text = None
                
                self.node.get_logger().info(">> ROBOT: " + str(text))
            
                wav = get_audio_file(text)
                success, message = self.play_audio_request(wav)

                self.node.get_logger().info(">> ROBOT: " + str(message))
                  
            rclpy.spin_once(self.node)
        
def main(args=None):
    rclpy.init(args=args)
    
    hri_tts = TTSlogic()
    hri_tts.spin()
    
    rclpy.shutdown()