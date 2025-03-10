import sounddevice as sd
import numpy as np
from scipy.ndimage import gaussian_filter1d

#from ..Direction.gui import DirectionDialog

#direction_dialog = DirectionDialog()

def normalized_cross_correlation(first_signal, second_signal):
    '''Calculates the NCC of first_signal and second_signal.
    
    Args:
        first_signal (Array: int): First signal.
        second_signal (Array: int): Second signal.
    
    Returns:
        ncc (float): The NCC of the signals.
    '''

    cross_correlation = np.correlate(first_signal, second_signal, mode='valid')[0]
    normA = np.linalg.norm(first_signal)
    normB = np.linalg.norm(second_signal)

    ncc = cross_correlation / (normA * normB)

    return ncc

def display_direction(direction, max_ncc, best_displacement):
    '''Displays the direction that the program has calculated, with some data.
    
    Args:
        direction (str): Direction of the sound (Left, Right or Center).
        max_ncc (float): Maximum NCC achieved.
        best_displacement (int): The displacement used for obtaining max_ncc.
    '''

    title, subtitle, subtitle2 = "", "", ""

    if direction == "Left":
        title = "⭮"
        subtitle = "Sonido a la izquierda"
    elif direction == "Right":
        title = "⭯"
        subtitle = "Sonido a la derecha"
    else:
        title = "•"
        subtitle = "Sonido perpendicular" # Añadir uno que sea sin sonido

    subtitle2 = "NCC: " + str(max_ncc) + ". Displacement: " + str(best_displacement) + "."

    #direction_dialog.display_dialog(title, subtitle, subtitle2)

def calculate_best_left_shift(shift_range, first_signal, second_signal):
    '''Finds the shift that gives maximum NCC value (where the signals overlap better).
    
    Args:
        shift_range (int): The range of shift to try.
        first_signal (Array: int): First signal.
        second_signal (Array: int): Second signal.

    Returns:
        best_displacement (int): The displacement used for obtaining max_ncc.
        max_ncc (float): Maximum NCC achieved.
    '''

    best_displacement, max_ncc = 0, 0
    for i in range(shift_range):
        shifted_first_signal = first_signal[i*2:]
        shifted_second_signal = second_signal[:len(second_signal)-i*2]

        ncc = normalized_cross_correlation(shifted_first_signal, shifted_second_signal)

        if ncc > max_ncc:
            best_displacement = i*2
            max_ncc = ncc
        
    return best_displacement, max_ncc

def determine_audio_location(microphone_left, microphone_right, shift_range=100, threshold=10):
    '''Given left and right microphone inputs, determines where is the source of sound.
    
    Args:
        microphone_left (Array: int): Captured signal on left microphone.
        microphone_right (Array: int): Captured signal on right microphone.
        shift_range (int): The range of shift to try
        threshold (int): Less than threshold will be considered as center.
    
    Returns:
        direction (str): Direction of the sound (Left, Right or Center).
        max_ncc (float): Maximum NCC achieved.
        best_displacement (int): The displacement used for obtaining max_ncc.
    '''
    # Importante convertir en float si no no hace NCC correctamente
    left_microphone_float = microphone_left.astype(float)
    right_microphone_float = microphone_right.astype(float)

    gaussian_left = gaussian_filter1d(left_microphone_float, 1) 
    gaussian_right = gaussian_filter1d(right_microphone_float, 1)

    best_shift_left, max_ncc_left = calculate_best_left_shift(shift_range, gaussian_left, gaussian_right)
    best_shift_right, max_ncc_right = calculate_best_left_shift(shift_range, gaussian_right, gaussian_left)

    best_displacement, max_ncc = 0, 0
    if max_ncc_left >= max_ncc_right:
        best_displacement, max_ncc = -best_shift_left, max_ncc_left
    else:
        best_displacement, max_ncc = best_shift_right, max_ncc_right

    direction = "Left" if best_displacement < -threshold else ("Right" if best_displacement > threshold else "Center")

    return direction, max_ncc, best_displacement

def record_audio(sample_rate, selected_device_index, record_duration):
    '''Records audio, determines sound source location and displays on screen.
    
    Args:
        sample_rate (int): Samples per second.
        selected_device_index (int): Index of the microphone.
        record_duration (int): Time of the record.
    '''

    recording = sd.rec(int(sample_rate * record_duration), samplerate=sample_rate, channels=2, device=selected_device_index, dtype=np.int16)
    sd.wait()

    left_microphone = np.asarray(recording[:, 0])
    right_microphone = np.asarray(recording[:, 1])

    # signals.plot_signals(left_microphone, right_microphone, gaussian_left, gaussian_right)

    # Esta funcion habria que hacerla en paralelo para que no se pierda sonido
    direction, max_ncc, best_displacement  = determine_audio_location(left_microphone, right_microphone) 
    
    display_direction(direction, max_ncc, best_displacement)
    
def localize_audio(sample_rate, selected_device_index, record_duration):
    '''Loops ``record_audio`` for determining sound source location multiple times.'''

    while True:
        record_audio(sample_rate, selected_device_index, record_duration) 