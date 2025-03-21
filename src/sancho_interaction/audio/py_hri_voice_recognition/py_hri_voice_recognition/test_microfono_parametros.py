import io
import time
import wave
import pyaudio
import numpy as np
device_index =0
maxInputChannels=0
defaultSampleRate=0
def listar_dispositivos():
    p = pyaudio.PyAudio()
    print("Dispositivos de entrada:")
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        if dev['maxInputChannels'] > 0:
            print(f"{i}: {dev['name']}")
    print("\nDispositivos de salida:")
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        if dev['maxOutputChannels'] > 0:
            print(f"{i}: {dev['name']}")
    p.terminate()

def mostrar_parametros(numero_dispositivo):
    p = pyaudio.PyAudio()
    global device_index
    global maxInputChannels
    global defaultSampleRate
    dev = p.get_device_info_by_index(numero_dispositivo)
    print(dev)
    print("Parámetros del dispositivo:")
    print(f"{numero_dispositivo}: {dev['name']}, Canales: {dev['maxInputChannels']}, Tasa de muestreo: {int(dev['defaultSampleRate'])}")
    device_index = numero_dispositivo
    maxInputChannels = dev['maxInputChannels']
    defaultSampleRate = int(dev['defaultSampleRate'])
    p.terminate()

def main():
    print("Lista de dispositivos de audio disponibles:")
    listar_dispositivos()
    print("-------------------------------------------")
    print("Parametros del dispositivo X:")
    mostrar_parametros(4)
    print("-------------------------------------------")
    # grabar_audio(device_index=4, sample_rate=defaultSampleRate, channels=maxInputChannels)
    

def grabar_audio(device_index, chunk_size=1024, sample_rate=48000, channels=2):
    p = pyaudio.PyAudio()
    print(device_index)
    stream = p.open(format=pyaudio.paInt16,
                    channels=channels,
                    rate=sample_rate,
                    input=True,
                    frames_per_buffer=chunk_size,
                    input_device_index=device_index)

    print("Grabando audio... (Presiona Ctrl+C para detener)")
    i = 0
    try:
        tiempo_inicial = time.time()  # Capturamos el tiempo inicial
        while True:
        # Aquí colocas el código que quieres ejecutar durante un segundo
            data = stream.read(chunk_size)
            audio_data = np.frombuffer(data, dtype=np.int16)
            i = i+1
            
            tiempo_actual = time.time()  # Capturamos el tiempo actual
            if tiempo_actual - tiempo_inicial >= 1:
                break  # Salimos del bucle si han pasado más de 1 segundo
                
        print(i)    
    except KeyboardInterrupt:
        print("\nGrabación detenida.")
    
    stream.stop_stream()
    stream.close()
    p.terminate()

def convert_to_wav(left_channel, right_channel, sample_rate):
    # Combinar los canales izquierdo y derecho en un arreglo estéreo de numpy
    audio_stereo = np.column_stack((left_channel, right_channel))
    
    # Convertir el arreglo de numpy a un objeto de bytes en formato WAV
    wav_bytes = io.BytesIO()
    with wave.open(wav_bytes, 'wb') as wf:
        # Configurar los parámetros del archivo WAV
        wf.setnchannels(2)  # Dos canales (estéreo)
        wf.setsampwidth(2)  # 2 bytes por muestra
        wf.setframerate(sample_rate)  # Frecuencia de muestreo
        
        # Convertir el audio a formato de bytes
        audio_bytes = audio_stereo.astype(np.int16).tobytes()
        
        # Escribir los datos de audio en el archivo WAV
        wf.writeframes(audio_bytes)
    
    # Obtener los bytes del objeto de BytesIO y devolverlos
    wav_bytes.seek(0)
    return wav_bytes.read()

if __name__ == "__main__":
    main()
