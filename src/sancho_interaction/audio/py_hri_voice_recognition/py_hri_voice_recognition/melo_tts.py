from melo.api import TTS
import time

speed = 1.0
device = 'cuda:0'
model = TTS(language='ES', device=device)

speaker_ids = model.hps.data.spk2id
output_path = '/tmp/tts_audio_es.wav'

def get_audio_file(text):
    a = time.time()
    wav = model.tts_to_file(text, speaker_ids['ES'], speed=speed)
    print("Tiempo en crear audio de un texto de " + str(len(text)) + " caracteres: " + str(time.time() - a))

    return wav