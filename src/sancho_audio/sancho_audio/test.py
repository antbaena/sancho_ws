import numpy as np
import sounddevice as sd
import torch
import torchaudio

# Cargar modelo Silero VAD
model, utils = torch.hub.load(
    repo_or_dir="snakers4/silero-vad", model="silero_vad", trust_repo=True
)
(get_speech_timestamps, _, _, _, _) = utils

# Configuración
SAMPLE_RATE = 16000
CHUNK_SIZE = 512  # 64 ms
CHANNELS = 1

# Inicializar el estado del modelo
model.reset_states()


def audio_callback(indata, frames, time, status):
    if status:
        print(f"Status: {status}")
    audio_chunk = torch.from_numpy(indata[:, 0].copy()).float()

    # Asegúrate de que esté en el rango [-1, 1]
    max_val = max(audio_chunk.abs().max().item(), 1e-6)
    audio_chunk = audio_chunk / max_val

    # Pasar al modelo
    is_speech = model(audio_chunk, SAMPLE_RATE).item()
    if is_speech:
        print("🗣️ Voz detectada")
    else:
        print("🤫 Silencio")


print("🎤 Escuchando... (Ctrl+C para detener)")
try:
    with sd.InputStream(
        channels=CHANNELS,
        samplerate=SAMPLE_RATE,
        blocksize=CHUNK_SIZE,
        dtype="float32",
        callback=audio_callback,
    ):
        while True:
            sd.sleep(1000)
except KeyboardInterrupt:
    print("\n🛑 Finalizado por el usuario.")
