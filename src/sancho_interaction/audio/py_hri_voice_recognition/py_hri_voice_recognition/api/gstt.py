import base64
import requests

def transcribe(audio_samples, sample_rate):
    audio_base64 = base64.b64encode(audio_samples).decode('utf-8')
    
    data = {
        "config": {
            "encoding": "LINEAR16",
            "sampleRateHertz": sample_rate,
            "languageCode": "es-ES"
        },
        "audio": {
            "content": audio_base64
        }
    }

    response = requests.post(
        "https://speech.googleapis.com/v1/speech:recognize?key=AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw",
        json=data
    )
    
    if response.status_code == 200 and response is not None:
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
        return None # If error