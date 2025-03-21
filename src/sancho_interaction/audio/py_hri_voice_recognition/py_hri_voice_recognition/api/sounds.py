import os

from .sound import load

dirname = os.path.dirname(os.path.dirname(__file__))

activationPath = 'install/py_hri_voice_recognition/share/py_hri_voice_recognition/sounds/activation_sound.wav'
timeOutPath = 'install/py_hri_voice_recognition/share/py_hri_voice_recognition/sounds/time_out_sound.wav'

ACTIVATION_SOUND = load(activationPath)
TIME_OUT_SOUND = load(timeOutPath)
