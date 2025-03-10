import pyaudio
import numpy as np
import rclpy
from rclpy.node import Node
from human_face_recognition_msgs.msg import ChunkMono, ChunkStereo

class MicrophoneCapturer(Node):
    """Ros2 Node for capturing audio from the microphone and publishing it as AudioChunk messages."""

    def __init__(self, device_index, chunk_size):
        """
        Initialize the MicrophoneCapturer node.

        Args: 
            device_index (int): The index of the audio input device to use.
            sample_rate (int): The sample rate of the audio to capture, in Hz.
            num_channels (int): The number of audio channels to capture.
            chunk_size (int): The size of the audio chunk to capture in each read from the input device.
        """
        super().__init__("microphone_capturer")
        self.get_logger().info("Inicializando microfono en el indice: "+ str(device_index))
                               
        p = pyaudio.PyAudio()
        dev = p.get_device_info_by_index(device_index)

        self.device_index = dev['index']
        self.sample_rate = int(dev['defaultSampleRate'])
        self.num_channels = dev['maxInputChannels']
        self.chunk_size = chunk_size

        p.terminate()

        self.publisher_stereo = self.create_publisher(ChunkStereo, 'AudioRaw/microphone/stereo', 10)
        self.publisher_mono = self.create_publisher(ChunkMono, 'AudioRaw/microphone/mono', 10)
        self.setup_microphone(self.device_index, self.sample_rate, self.num_channels, self.chunk_size)
        
        self.run()
    
    def run(self):
        """
        Capture audio from the microphone and publish it as AudioChunk messages.
        """
        chunk_stereo = ChunkStereo()
        chunk_mono = ChunkMono()

        first = True
        while True:
            try:                
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                data = np.frombuffer(data, dtype=np.int16)

                audio_mix =np.ndarray.tolist(data)

                if self.num_channels == 1:
                    chunk_mono = audio_mix

                elif self.num_channels >= 2:
                    chunk_stereo.chunk_left = audio_mix[::2]
                    chunk_stereo.chunk_right  = audio_mix[1::2]

                    chunk_mono.chunk_mono = audio_mix[::2] # Probar a hacer algun metodo de mono to stereo

                    self.publisher_stereo.publish(chunk_stereo)

                if first:
                    self.get_logger().info("Stereo audio frame published (if num channels is 1 or 2)")
                    first = False
                
                self.publisher_mono.publish(chunk_mono)
            except Exception as e:
                 self.get_logger().info("FATAL ERROR")
    
    def setup_microphone(self, device_index, sample_rate, num_channels, chunk_size):
        """
        Setup the microphone for audio capture.
        
        Args: 
            device_index (int): The index of the audio input device to use.
            sample_rate (int): The sample rate of the audio to capture, in Hz.
            num_channels (int): The number of audio channels to capture.
            chunk_size (int): The size of the audio chunk to capture in each read from the input device.
        """
        p = pyaudio.PyAudio()
        self.stream = p.open(format=pyaudio.paInt16,
                            channels=num_channels,
                            rate=sample_rate,
                            input=True,
                            frames_per_buffer=chunk_size,
                            input_device_index=device_index)

def get_microphone_index(name):
    p = pyaudio.PyAudio()
    microphone_index = -1  # Inicializa el índice del micrófono como -1

    # Itera sobre todos los dispositivos de audio
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        if dev['maxInputChannels'] > 0:  # Solo dispositivos de entrada
            # Imprimir el nombre del dispositivo
            print(f"{i}: {dev['name']}")
            # Comparar el nombre del dispositivo con el nombre proporcionado (ignorando mayúsculas y minúsculas)
            if name.lower() in dev['name'].lower():
                microphone_index = i  # Guarda el índice del micrófono encontrado

    return microphone_index  # Retorna el índice del micrófono, o -1 si no se encuentra
     
def main(args=None):
    mic_name="orbbec"
    rclpy.init(args=args)
    microphone_index = get_microphone_index(mic_name)
    if microphone_index == -1:
        raise RuntimeError(f"Micrófono {mic_name} no encontrado.") 
    microphone_capturer = MicrophoneCapturer(device_index=microphone_index, chunk_size=1024)
    rclpy.spin(microphone_capturer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()