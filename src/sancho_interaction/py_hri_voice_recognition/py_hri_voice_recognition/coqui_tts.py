import asyncio
from TTS.api import TTS
import sounddevice as sd
import numpy as np

# Inicializar el modelo con voz masculina (CSS10)
tts = TTS(model_name="tts_models/es/css10/vits", progress_bar=False, gpu=False)


# Diccionario para reemplazar caracteres especiales por palabras
replacements = {
    '0': 'cero',
    '1': 'uno',
    '2': 'dos',
    '3': 'tres',
    '4': 'cuatro',
    '5': 'cinco',
    '6': 'seis',
    '7': 'siete',
    '8': 'ocho',
    '9': 'nueve',
    ':': 'dos puntos',
    ';': 'punto y coma',
    '@': 'arroba',
    '#': 'numeral',
    '$': 'dólar',
    '%': 'por ciento',
    '^': 'caret',
    '&': 'y',
    '*': 'asterisco',
    '(': 'paréntesis abierto',
    ')': 'paréntesis cerrado',
    '-': 'guion',
    '_': 'guion bajo',
    '+': 'más',
    '=': 'igual',
    '<': 'menor que',
    '>': 'mayor que',
}

# Función que será llamada normalmente sin await
def read_text_async(text):
    # Generar las ondas de audio de manera asíncrona
    for key, value in replacements.items():
        text = text.replace(key, value)
    wav = tts.tts(text)
   
    # Reproducir el audio
    sd.play(wav / np.max(np.abs(wav)), samplerate=24000)
    sd.wait()
    
def get_audio_file(text):
    # Generar las ondas de audio de manera asíncrona
    for key, value in replacements.items():
        text = text.replace(key, value)
    wav = tts.tts(text)
    return wav