import time

import rclpy
from rclpy.node import Node

from human_face_recognition_msgs.srv import AudioRecognition
from std_msgs.msg import String

import soundfile as sf
import librosa
import librosa.display
import numpy as np
from pyannote.audio import Pipeline
import whisper
from pydub import AudioSegment
import tempfile
import io
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"  # Usar la GPU 0


class WhisperSTT(Node):

    def __init__(self):
        super().__init__("whisper_stt")
        self.model = whisper.load_model("medium")
        self.stt_service = self.create_service(AudioRecognition, "audio/recognition", self.audio_recognition)
        self.transcription_topic = self.create_publisher(String, "audio/transcription", 5)
        
        self.pipeline = Pipeline.from_pretrained("pyannote/voice-activity-detection",
            use_auth_token="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")

        self.get_logger().info('STT inicializado correctamente')
    
    def is_speech(self, audio, sample_rate):
        has_speech = False
        audio_samples = np.array(audio, dtype=np.int16) 
        audio = AudioSegment(
            audio_samples.tobytes(), 
            frame_rate=sample_rate,
            sample_width=audio_samples.dtype.itemsize, 
            channels=1
        )
        
        temp_wav_path = "/tmp/temp_audio_recog.wav"
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
        
        output = self.pipeline(temp_wav_path)
        has_speech = any(output.get_timeline().support())

        return has_speech

    def audio_recognition(self, request, response):
        start_time = time.time()

        audio = request.audio
        sample_rate = request.sample_rate

        print("Received " + str(len(list(request.audio)) / sample_rate) + " seconds of audio.")
              
        if not self.is_speech(audio, sample_rate):
            print("The audio segment is not human speech.")
            response.text = String(data=str(None))
            return response

        audio_bytes = bytearray(audio)
        audio_file_wav = convert_bytes_to_wav(audio_bytes, sample_rate)
        
        if audio_file_wav:
            result = self.model.transcribe(audio_file_wav, language="es")
            text = result["text"]
            if len(text) == 0:
                text = None

            response.text = String(data=str(text))
            elapsed_time = time.time() - start_time

            if text is not None:
                print("Transcription after " + str(round(elapsed_time, 3)) + " seconds: " + text)
                self.transcription_topic.publish(response.text)
            else:
                print("Transcription is None (Took " + str(round(elapsed_time, 3)) + " seconds)")
            
        return response

def main(args=None):
    rclpy.init(args=args)

    whisper_stt = WhisperSTT()

    rclpy.spin(whisper_stt)
    rclpy.shutdown()
    
def convert_bytes_to_wav(audio_bytes, sample_rate):
    try:
        # Parámetros de audio
        sample_width = 2  # Tamaño de muestra en bytes (por ejemplo, 2 para audio de 16 bits)
        channels = 1  # Número de canales (1 para mono, 2 para estéreo)
        
        # Crear un objeto BytesIO desde los bytes del audio
        audio_stream = io.BytesIO(audio_bytes)
        
        # Leer los datos de audio como raw PCM
        data = audio_stream.read()
        
        # Crear un objeto AudioSegment a partir de los datos raw PCM
        audio = AudioSegment(
            data=data,
            sample_width=sample_width,
            frame_rate=sample_rate,
            channels=channels
        )
        
        # Crear un archivo temporal
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        temp_path = temp_file.name
        
        # Exportar el archivo como WAV
        audio.export(temp_path, format="wav")
        
        # Cerrar el archivo temporal
        temp_file.close()
        
        return temp_path
    except Exception as e:
        print(f"Error al convertir bytes a WAV: {e}")
        return None
