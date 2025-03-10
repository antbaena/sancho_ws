from melo.api import TTS

# Speed is adjustable
speed = 1.0

# CPU is sufficient for real-time inference.
# You can also change to cuda:0
device = 'cpu'

text = "El resplandor del sol acaricia las olas, pintando el cielo con una paleta deslumbrante."
model = TTS(language='ES', device=device)
speaker_ids = model.hps.data.spk2id

output_path = 'es.wav'
model.tts_to_file(text, speaker_ids['ES'], output_path, speed=speed)
