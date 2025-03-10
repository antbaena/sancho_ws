from gtts import gTTS
from io import BytesIO
from pygame import mixer
import time
import threading
from pydub import AudioSegment
# Transformar a objeto (variables globales -> code smell casi siempre)

ACCENT_MEXICO = 'com.mx'
ACCENT_SPAIN = 'es'
ACCENT_US = 'us'

_monitor = threading.Condition()
_speaking = False

# Meter atributo override que si es true borra todo lo que se este diciendo y se dice lo nuevo
# Probar la reproduccion del audio con sound.py
def read_text(text, language='es', accent=ACCENT_SPAIN):
    '''Reads a text using Google Text To Speech tool.
    
    Note:
        If a text is being readed and the user asks for another tts,
        the second tts will wait until the first one ends.

    Args:
        text (str): The text to be readed.
        language (str): Language of the reader.
        accent (str): Accent of the reader.
    '''

    global _speaking 
    
    with _monitor:
        while _speaking:
            _monitor.wait()
        _speaking = True

        mp3_fp = BytesIO()
        tts = gTTS(text, lang=language, tld=accent)
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)
        mixer.music.load(mp3_fp, "mp3")
        mixer.music.play()

        while mixer.music.get_busy():
            time.sleep(0.1)

        _speaking = False
        _monitor.notify_all()
    

def read_text_async(text, language='es', accent=ACCENT_SPAIN):
    '''Reads the text using read_text function asynchronously.'''

    speakThread = threading.Thread(target=read_text, args=(text,language,accent))
    speakThread.start()


def get_audio_file(text, language='es', accent=ACCENT_SPAIN):
    """
    Genera un archivo de audio WAV en memoria a partir del texto proporcionado
    utilizando gTTS y devuelve los bytes del archivo.
    
    Args:
        text (str): El texto a convertir en audio.
        language (str): El idioma del texto.
        accent (str): El acento del idioma.

    Returns:
        bytes: Los bytes del archivo de audio WAV.
    """
    # Generar el audio con gTTS y almacenarlo en un objeto BytesIO como MP3
    mp3_fp = BytesIO()
    tts = gTTS(text, lang=language, tld=accent)
    tts.write_to_fp(mp3_fp)
    mp3_fp.seek(0)

    # Convertir el MP3 en WAV usando pydub
    audio = AudioSegment.from_mp3(mp3_fp)
    wav_fp = BytesIO()
    audio.export(wav_fp, format="wav")
    wav_fp.seek(0)

    # Retornar los bytes del archivo WAV
    return wav_fp.read()


mixer.init()