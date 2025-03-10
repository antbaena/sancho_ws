from TTS.api import TTS
import sounddevice as sd
import numpy as np
import time

tts = TTS(model_name="tts_models/es/css10/vits", progress_bar=False, gpu=False)

a = time.time()
wav = tts.tts(text="hola que tal tengo nueva voz")
print(time.time() -a)
sd.play(wav / np.max(np.abs(wav)), samplerate=24000)
sd.wait()

a = time.time()
wav = tts.tts(text="El tiempo en málaga son 40º a las 2:34 horas de la mañana, vamos a morir. De calor")
print(time.time() -a)
sd.play(wav / np.max(np.abs(wav)), samplerate=24000)
sd.wait()
