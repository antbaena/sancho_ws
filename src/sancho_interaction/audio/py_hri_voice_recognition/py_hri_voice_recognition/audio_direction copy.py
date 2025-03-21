import numpy as np

import rclpy
from rclpy.node import Node

from human_face_recognition_msgs.msg import ChunkStereo
from human_face_recognition_msgs.srv import AudioLocation

from .api.sound_localizer import determine_audio_location
from pyannote.audio import Pipeline
from pydub import AudioSegment
import numpy as np
from io import BytesIO


class AudioDirection(Node):

    def __init__(self, sample_rate=48000, distance_mics=0.1225, direction_chunk_seconds=1):
        super().__init__("audio_direction")
        
        self.sample_rate = sample_rate
        self.speed_sound = 343
        self.distance_mics = distance_mics
        
        self.direction_chunk_seconds = direction_chunk_seconds # Seconds per chunk
        
        self.subscription = self.create_subscription(ChunkStereo, 'AudioRaw/microphone/stereo', self.listener_callback, 10)
        self.direction_client = self.create_service(AudioLocation, 'audio_location', self.audio_location)
        
        self.audio_left = []
        self.audio_right = []

        # Instantiate the pipeline with your Hugging Face access token
        self.pipeline = Pipeline.from_pretrained("pyannote/voice-activity-detection",
                                            use_auth_token="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")

        # Apply the pipeline to the audio file
        


    def audio_average_intensity(self, audio):
        average_intensity = np.mean(np.abs(audio))
        if average_intensity < 0:
            average_intensity = 32767
        return average_intensity
    
    def audio_location(self, request, response):
        audio_samples = np.array(self.audio_left, dtype=np.int16) 
        audio = AudioSegment(
            audio_samples.tobytes(), 
            frame_rate=self.sample_rate,
            sample_width=audio_samples.dtype.itemsize, 
            channels=1
        )

        # Guarda el AudioSegment en un archivo temporal en formato WAV
        temp_wav_path = "/tmp/temp_audio.wav"
        audio.export(temp_wav_path, format="wav")

        output = self.pipeline(temp_wav_path)
        has_speech = any(output.get_timeline().support())
        if(has_speech):
            _, _, best_shift = determine_audio_location(microphone_left=np.asarray(self.audio_left),microphone_right=np.asarray(self.audio_right),threshold=3)
            audio_intensity = self.audio_average_intensity(self.audio_left)

            time_variation = -best_shift / self.sample_rate # seconds
            sin_degree = ((self.speed_sound * time_variation) / self.distance_mics)
            sin_degree = 1 if sin_degree > 1 else (-1 if sin_degree < -1 else sin_degree)

            angle_rad = np.arcsin(sin_degree)
            depth = np.sqrt(np.abs((self.distance_mics*self.distance_mics)-((time_variation*self.speed_sound )*(time_variation*self.speed_sound ))))
            response.intensity = audio_intensity
            response.angle_rad = angle_rad
        else:
            response.intensity = -1.0
            response.angle_rad = 0.0
        
        return response
    
    def listener_callback(self, msg):
        self.audio_left = self.audio_left + list([np.int16(x) for x in msg.chunk_left])
        self.audio_right = self.audio_right + list([np.int16(x) for x in msg.chunk_right])
        
        chunk_samples = len(msg.chunk_left)
        max_samples = self.sample_rate * self.direction_chunk_seconds
        if(len(self.audio_left) > max_samples):                
            self.audio_left = self.audio_left[chunk_samples:]
            self.audio_right = self.audio_right[chunk_samples:]
        
def main(args=None):
    rclpy.init(args=args)
    
    audio_direction = AudioDirection()
    
    rclpy.spin(audio_direction)
    rclpy.shutdown()