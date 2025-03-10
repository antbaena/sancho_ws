from melo.api import TTS
from playsound import playsound
import time 

speed = 1.0
# CPU is sufficient for real-time inference.
# You can also change to cuda:0
device = 'cpu'

model = TTS(language='ES', device=device)
speaker_ids = model.hps.data.spk2id
print(speaker_ids)

output_path = 'audio_temp.wav'

a = time.time()
model.tts_to_file("hola que tal tengo nueva voz", speaker_ids['ES'], output_path, speed=speed)
print(time.time() -a)
playsound(output_path)

a = time.time()
model.tts_to_file("El tiempo en málaga son 40º a las 2:34 horas de la mañana, vamos a morir. De calor", speaker_ids['ES'], output_path, speed=speed)
print(time.time() -a)

# Reproduce el archivo WAV generado
playsound(output_path)
