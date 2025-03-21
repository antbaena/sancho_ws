from melo.api import TTS
import numpy as np
import sounddevice as sd
import threading
import time

# Variables globales para gestionar el estado de reproducción
_monitor = threading.Condition()
_speaking = False

# Configuración del dispositivo
device = 'cpu'
language = 'ES'

def read_text(text, speaker='ES'):
    '''Lee un texto usando MeloTTS y reproduce el audio con la voz especificada.
    
    Args:
        text (str): El texto a leer.
        speaker (str): El ID del hablante (voz) a utilizar.
    '''
    global _speaking
    
    with _monitor:
        while _speaking:
            _monitor.wait()
        _speaking = True

        # Inicializa el modelo
        model = TTS(language=language, device=device)
        speaker_ids = model.hps.data.spk2id

        # Comprueba si el hablante existe
        if speaker not in speaker_ids:
            print(f"Hablante '{speaker}' no encontrado. Usando el hablante por defecto.")
            speaker = 'ES'  # O elige un ID por defecto

        # Genera el audio en memoria
        audio = model.tts(text, speaker_ids[speaker])
        
        # Extrae el audio como un arreglo de NumPy
        audio_np = np.array(audio)

        # Define la tasa de muestreo
        sample_rate = model.hps.data.sampling_rate
        
        # Reproduce el audio directamente
        sd.play(audio_np, samplerate=sample_rate)

        # Espera a que termine la reproducción
        while sd.get_stream().active:
            time.sleep(0.1)

        _speaking = False
        _monitor.notify_all()

def read_text_async(text, speaker='ES'):
    '''Lee el texto de forma asíncrona con la voz especificada.'''
    speak_thread = threading.Thread(target=read_text, args=(text, speaker))
    speak_thread.start()

# Ejemplo de uso
if __name__ == "__main__":
    text_to_read = "El resplandor del sol acaricia las olas, pintando el cielo con una paleta deslumbrante."
    
    # Elige el hablante (puedes cambiar 'ES' por otro ID de hablante disponible)
    chosen_speaker = 'ES'  # Cambia esto al ID del hablante que desees usar
    
    # Inicializa el hilo de reproducción
    read_text_async(text_to_read, chosen_speaker)

    # Puedes añadir más textos para que se lean asíncronamente
