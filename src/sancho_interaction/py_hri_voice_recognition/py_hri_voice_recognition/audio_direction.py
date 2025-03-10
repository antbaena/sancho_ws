import numpy as np
import json

import rclpy
from rclpy.node import Node

from human_face_recognition_msgs.msg import ChunkStereo
from human_face_recognition_msgs.srv import AudioLocation

from .api.sound_localizer import determine_audio_location
from pyannote.audio import Pipeline
from pydub import AudioSegment
import numpy as np

from std_msgs.msg import String

import soundfile as sf
import librosa
import librosa.display
import numpy as np


class AudioDirection(Node):

    def __init__(self, sample_rate=48000, distance_mics=0.1225, direction_chunk_seconds=1):
        super().__init__("audio_direction")
        
        self.sample_rate = sample_rate
        self.speed_sound = 343
        self.distance_mics = distance_mics
        
        self.direction_chunk_seconds = direction_chunk_seconds # Seconds per chunk
        
        self.subscription = self.create_subscription(ChunkStereo, 'AudioRaw/microphone/stereo', self.listener_callback, 10)
        self.direction_client = self.create_service(AudioLocation, 'audio_location', self.audio_location)
        self.publisher_human_voice_pan = self.create_publisher(String, "audio/human_voice_pan", 1)
        
        self.audio_left = []
        self.audio_right = []

        # Instantiate the pipeline with your Hugging Face access token
        self.pipeline = Pipeline.from_pretrained("pyannote/voice-activity-detection",
                                            use_auth_token="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")

        # Apply the pipeline to the audio file
        print("Audio Direction inicializado correctamente")


    def audio_average_intensity(self, audio):
        average_intensity = np.mean(np.abs(audio))
        if average_intensity < 0:
            average_intensity = 32767
        return average_intensity
    
    def audio_location(self, request, response, intensity_threshold=300):
        has_speech = False
        audio_samples = np.array(self.audio_left, dtype=np.int16) 
        audio = AudioSegment(
            audio_samples.tobytes(), 
            frame_rate=self.sample_rate,
            sample_width=audio_samples.dtype.itemsize, 
            channels=1
        )
        
        temp_wav_path = "/tmp/temp_audio.wav"
        audio.export(temp_wav_path, format="wav")
        y, sr = librosa.load(temp_wav_path, sr=None)

        # Obtener la STFT (Short-Time Fourier Transform)
        D = librosa.stft(y)

        # Convertir a magnitud
        S_full, phase = librosa.magphase(D)

        # Calcular el espectro de ruido (puedes usar una parte silenciosa del audio)
        offset = 1000
        S_noise = np.mean(S_full[:, :offset], axis=1)

        # Substraer el espectro de ruido
        S_denoised = S_full - S_noise[:, np.newaxis]

        # Volver a la señal temporal
        D_denoised = S_denoised * phase
        y_denoised = librosa.istft(D_denoised)

        # Guardar el audio denoised
        sf.write(temp_wav_path, y_denoised, sr)
        
        
        if self.audio_average_intensity(np.array(AudioSegment.from_wav(temp_wav_path).get_array_of_samples(), dtype=np.int16)) >intensity_threshold:  
            output = self.pipeline(temp_wav_path)
            has_speech = any(output.get_timeline().support())      
        
        if(has_speech):
            longest_segment = None
            max_duration = 0
            
            for segment in output.get_timeline().support():
                start_time = segment.start
                end_time = segment.end
                duration = end_time - start_time
                if duration > max_duration:
                    max_duration = duration
                    longest_segment = segment

            if longest_segment:
                start_time = longest_segment.start
                end_time = longest_segment.end
                
                # Convert start and end times to sample indices
                start_sample = int(start_time * self.sample_rate)
                end_sample = int(end_time * self.sample_rate)
                
                # Extract the audio segments
                audio_left_segment = self.audio_left[start_sample:end_sample]
                audio_right_segment = self.audio_right[start_sample:end_sample]
                
                # Use the segments in your function
                _, _, best_shift = determine_audio_location(microphone_left=np.asarray(audio_left_segment),
                                                            microphone_right=np.asarray(audio_right_segment),
                                                            threshold=3)
                audio_intensity = self.audio_average_intensity(audio_left_segment)

                time_variation = -best_shift / self.sample_rate # seconds
                sin_degree = ((self.speed_sound * time_variation) / self.distance_mics)
                sin_degree = 1 if sin_degree > 1 else (-1 if sin_degree < -1 else sin_degree)

                angle_rad = np.arcsin(sin_degree)
                depth = np.sqrt(np.abs((self.distance_mics*self.distance_mics)-((time_variation*self.speed_sound )*(time_variation*self.speed_sound ))))
                response.intensity = audio_intensity
                response.angle_rad = angle_rad
                self.audio_left = []
                self.audio_right =[]
        else:
            response.intensity = -1.0
            response.angle_rad = 0.0
        
        self.publisher_human_voice_pan.publish(String(data=json.dumps({
            "intensity": response.intensity,
            "angle_rad": response.angle_rad
        })))

        return response
    
    def listener_callback(self, msg):
        self.audio_left = self.audio_left + list([np.int16(x) for x in msg.chunk_left])
        self.audio_right = self.audio_right + list([np.int16(x) for x in msg.chunk_right])
        
        chunk_samples = len(msg.chunk_left)
        max_samples = self.sample_rate * self.direction_chunk_seconds
        if(len(self.audio_left) > max_samples):                
            self.audio_left = self.audio_left[chunk_samples:]
            self.audio_right = self.audio_right[chunk_samples:]
            
        #     max_samples = self.sample_rate * self.direction_chunk_seconds
        # if(len(self.audio_left) >= max_samples):         
        #     chunk_samples = len(msg.chunk_left)       
        #     self.audio_left = self.audio_left[chunk_samples:]
        #     self.audio_right = self.audio_right[chunk_samples:]
        #     self.audio_left = self.audio_left + list([np.int16(x) for x in msg.chunk_left])
        #     self.audio_right = self.audio_right + list([np.int16(x) for x in msg.chunk_right])
        # else: #Inicial para el llenado de los buffers
        #     self.audio_left = self.audio_left + list([np.int16(x) for x in msg.chunk_left])
        #     self.audio_right = self.audio_right + list([np.int16(x) for x in msg.chunk_rig
        
def main(args=None):
    rclpy.init(args=args)
    
    audio_direction = AudioDirection()
    
    rclpy.spin(audio_direction)
    rclpy.shutdown()