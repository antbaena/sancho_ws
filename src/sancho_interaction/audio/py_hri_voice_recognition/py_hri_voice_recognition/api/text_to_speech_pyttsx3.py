import pyttsx3
import threading

monitor = threading.Condition()
speaking = False

def init_motor(wpm=150):
    '''Creates the pyttsx3 motor and sets properties.
    
    Args:
        wpm (int): Reading words per minute.
    '''

    motor = pyttsx3.init()
    motor.setProperty('rate', wpm)  # Words per minute
    motor.setProperty('voice', 'spanish')

    return motor

motor = init_motor()

def read_text(text, wpm=150):
    '''Reads a text using pyttsx3.
    
    Note:
        If a text is being read and the user asks for another tts,
        the second tts will wait until the first one ends.

    Args:
        text (str): The text to be read.
        wpm (int): Words per minute.
    '''

    global speaking

    print(">> ROBOT: " + text)
    with monitor:
        while speaking:
            monitor.wait()
        speaking = True

        motor.setProperty('rate', wpm)  # Update WPM if changed
        motor.say(text)
        motor.runAndWait()

        speaking = False
        monitor.notify_all()

def read_text_async(text):
    '''Reads the text using read_text function asynchronously.'''

    speakThread = threading.Thread(target=read_text, args=(text,))
    speakThread.start()
