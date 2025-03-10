import base64
import requests
import numpy as np

def transcribe(audio_samples):
#     # Codificar el audio en base64
#     audio_samples_int = np.int16(np.array(audio_samples) * 32767)

# # Convertir las muestras de audio a un objeto de bytes
#     audio_bytes = audio_samples_int.tobytes()
    audio_base64 = base64.b64encode(audio_samples).decode('utf-8')
    
    # Construir el cuerpo de la solicitud HTTP
    data = {
        "config": {
            "encoding": "LINEAR16",
            "sampleRateHertz": 48000,  # Ajusta el sample rate según tu audio
            "languageCode": "es-ES"  # Lenguaje del audio
        },
        "audio": {
            "content": audio_base64
        }
    }
    # Realizar la solicitud POST a la API de Google Speech-to-Text
    response = requests.post(
        "https://speech.googleapis.com/v1/speech:recognize?key=AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw",
        json=data
    )
    
    
    # Procesar la respuesta
    if response.status_code == 200:
        result = response.json()
        if 'results' in result:
            best_transcript = max(
                (alternative for result in result['results'] for alternative in result['alternatives']),
                key=lambda alternative: alternative['confidence']
            )['transcript']
            return best_transcript
        else:
            return None
    else:
        return f"Error en la solicitud: {response.status_code} - {response.text}"
