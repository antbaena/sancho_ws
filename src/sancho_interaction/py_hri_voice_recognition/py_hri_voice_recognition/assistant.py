from datetime import datetime
import time
import requests
import json
import re
import unidecode
from openai import OpenAI
import pywhatkit
import wikipedia
from bs4 import BeautifulSoup
from queue import Queue
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from .api.sound import play
from .api.sounds import ACTIVATION_SOUND, TIME_OUT_SOUND
from .api.utils import translate

from human_face_recognition_msgs.srv import HelperMode, GetString


class Asssistant(Node):

    def __init__(self, name, available_commands):
        super().__init__("assistant")
        load_dotenv()
        # Obtener la clave API desde la variable de entorno
        self.api_key = os.getenv("OPENAI_API_KEY")
        
        self.Openai_client = OpenAI(
            organization='org-h9Nbg0kOMC5PlLNJz9tYEEC8',
            project='proj_1cFk2bZiwHfy6ThtTZNINDvt',
            )
        self.name = name
        #self.accent = ACCENT_SPAIN
        self.said_name = False
        self.actual_people = {}
        self.available_commands = available_commands
        self.text_queue = Queue(maxsize=1)

        self.subscription = self.create_subscription(String, 'audio/assistant/text', self.text_callback, 10)
        self.sub_actual_people = self.create_subscription(String, "robot/info/actual_people", self.actual_people_callback, 1)
        self.helper_mode_client = self.create_client(HelperMode, 'audio/helper/mode')
        while not self.helper_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Helper mode service not available, waiting again...')

        self.input_tts = self.create_publisher(String, 'input_tts', 10)
        self.command_actions = {
            "buscar": lambda arg: self.search(arg),
            "traducir a inglés": lambda arg: self.translate_to_english(arg),
            "tiempo actual": lambda _: self.actual_weather(),
            "cambiar nombre": lambda arg: self.change_name(arg),
            "nombre del asistente": lambda _: self.assistant_name(),
            "edad actual": lambda _: self.whats_my_age(),
            "hora actual": lambda _: self.actual_hour(),
            "nada": lambda _: self.nothing(),
            "quién soy": lambda _: self.who_am_i(),
            "repetir texto": lambda arg: self.repeat_text(arg),
            "reproducir video": lambda arg: self.reproduce_video_youtube(arg),
            "salir": lambda _: self.exit_assistant(),
            "comando desconocido": lambda _: self.unknown_command(),
            "no puedo ayudarte": lambda _: self.cant_help(),
        }

    def read_text(self, text):
        #msg = [text, self.accent]
        #msg_json = json.dumps(msg)
        
        self.input_tts.publish(String(data=text))
    
    def actual_people_callback(self, msg):
        self.actual_people = json.loads(msg.data)

    def text_callback(self, msg):
        if self.text_queue.qsize() < 1:
            self.text_queue.put(msg)
        else:
            self.get_logger().info("Text Queue IS FULL!!!")

    def helper_mode_request(self, mode):
        helper_mode_request = HelperMode.Request()

        helper_mode_request.mode = mode

        future_helper_mode = self.helper_mode_client.call_async(helper_mode_request)
        rclpy.spin_until_future_complete(self, future_helper_mode)
        result_helper_mode = future_helper_mode.result()

        return result_helper_mode.result

    def process_text(self, text):
        """Process the user's speech and determine if the assistant should be activated.

        The function checks whether the user has mentioned the assistant's name and if the assistant has been
        previously activated. If the user has not called the assistant yet, the function ignores the input.

        Args:
            text (str): The speech input from the user.
        """

        text_processed = text.strip().lower()
        text_processed = unidecode.unidecode(text_processed)
        text_processed = re.sub(r'[^a-z\s]', '', text_processed)
        print(">> USUARIO: " + text_processed + " (" + text + ")")

        if (self.name.lower() in text_processed and not self.said_name) or (self.name.lower() == text_processed and self.said_name):
            self.said_name = True
            self.helper_mode_request(1) # Listen for command
            play(ACTIVATION_SOUND)
        elif self.said_name:
            self.said_name = False
            self.helper_mode_request(0) # Listen for name
            most_similar_command, argument = self.process_command_llm(text_processed)
            if(most_similar_command is not None):
                self.execute_command(most_similar_command, argument)
    def process_command_llm(self, command):
            # Directivas adicionales para el comportamiento del robot
        messages = [
            {"role": "system", "content": (
                "Eres un robot asistente que responde a comandos de voz. "
                "Tu objetivo es entender la intención del usuario y encontrar "
                "el comando más similar entre las opciones disponibles. "
                "Si el comando incluye un argumento, identifica el argumento "
                "y devuélvelo junto con el comando. Por ejemplo, si el comando "
                "es 'buscar en Wikipedia la Torre Eiffel', el comando más similar "
                "sería 'buscar' y el argumento sería 'la Torre Eiffel'."
                "EL formato seria : <comando>.argumento:<arguemnto si existe>"
                "si no hay argumento,devuelve solo el comando de la forma: <comando>"
            )},

            # Formatea la solicitud para la API de ChatGPT
            {"role": "user", "content": f"Comando del usuario: '{command}'. " \
                    f"\nOpciones disponibles: {', '.join(self.available_commands)}.\n" \
                    "¿Cuál es el comando más similar para ejecutar y cuál es el argumento, si existe?"
            }
        ]
        # Llama a la API de ChatGPT
        completion = self.Openai_client.chat.completions.create(
            model="gpt-4o",
            messages=messages
        )

        # Extrae el texto de la respuesta
        response_text = completion.choices[0].message.content.strip()
        print(response_text)

        # Divide la respuesta en dos partes: el comando y el argumento
        if ".argumento:" in response_text.lower():
            parts = response_text.split(".argumento:", 1)
            most_similar_command = parts[0].strip()
            argument = parts[1].strip()
        else:
            most_similar_command = response_text
            argument = None

        return most_similar_command, argument
    
    def execute_command(self, most_similar_command, argument):
        # Buscar la función correspondiente en el diccionario
        action = self.command_actions.get(most_similar_command)

        # Verificar si existe una función para el comando identificado
        if action:
            # Ejecutar la función con o sin argumento según corresponda
            if argument:
                result = action(argument)
            else:
                result = action(None)
            # Retornar el resultado de la acción
            return f"Ejecutando: {most_similar_command}\nResultado: {result}"
        else:
            # Si no se encuentra el comando, devolver un mensaje de error
            return "No se encontró una acción para el comando especificado."
    
    def process_command(self, command, avaliable_commands):
        """Process the user's speech as a command.

        Note:
            If adding a new command, please, follow the same style, do not add the command
            function in this method, create a new private function and delegate the behaviour there.

        Args:
            command (str): The command as a string.
        """

        if command.startswith('busca'):  # Busca en Wikipedia lo que le digas
            topic = command.replace('busca', '')
            self.search(topic)
        elif command.startswith('cambiar nombre'):  # Cambia el nombre del asistente
            words = command.split()  # Divide la frase en palabras
            last_word = words[-1]  # Obtiene la última palabra
            self.change_name(last_word)  # Cambia el nombre con la última palabra
        elif 'como te llamas' == command or 'cómo te llamas' == command:  # Pregunta por el nombre del asistente
            self.assistant_name()
        elif 'edad' == command:  # Dice la edad de la persona que esta viendo
            self.whats_my_age()
        elif 'hora' in command:  # Dice la hora actual
            self.actual_hour()
        elif 'nada' == command:  # En caso de que no quieras nada
            self.nothing()
        elif 'quien soy' == command or 'quién soy' == command:  # Dice tu nombre en función de lo que reconoce
            self.who_am_i()
        elif command.startswith('repite'):  # Repite la frase que digas
            text = command.replace('repite ', '')
            self.repeat_text(text)
        elif command.startswith('reproduce'):  # Reproduce una canción en YouTube
            video = command.replace('reproduce', '')
            self.reproduce_video_youtube(video)
        elif 'salir' == command:  # Finaliza la ejecución
            self.exit_assistant()
        elif "tiempo" in command:
            self.actual_weather()
        elif command.startswith("traduce"):
            text = command.replace('traduce ', '')
            
            self.translate_to_english(text)
        else:  # En caso de no reconocer ningun comando
            self.unknown_command()

    def search(self, topic):
        """Search a topic on Wikipedia.
        
        Args:
            topic (str): The topic to be searched.
        """

        wikipedia.set_lang("es")
        try:
            info = wikipedia.summary(topic, 1)
            if info is None or len(info) == 0:
                raise Exception()
            self.read_text(info)
        except Exception:
            self.read_text("No he encontrado nada relacionado con tu busqueda")

    def translate_to_english(self, text):
        english_text = translate(text, "es", "en")
        self.read_text(english_text)

    def actual_weather(self):
        ciudad = "Malaga"
        api_key = "9ee89e6c536f992e3b25f22ae34e4124"
        result = f"Lo siento, ha habido un problema y no he podido obtener el tiempo en {ciudad}"
        try:
            url = f"http://api.openweathermap.org/data/2.5/weather?q={ciudad}&appid={api_key}&units=metric"
            response = requests.get(url)
            data = response.json()

            if response.status_code == 200:
                clima = data['weather'][0]['description']
                temperatura = data['main']['temp']

                clima_traducido = translate(clima, 'en', 'es')
                hora_actual = datetime.now().strftime("%H:%M:%S")

                result = f"El tiempo en {ciudad} a las {hora_actual} es: {clima_traducido} y hace una temperatura de {temperatura} grados celsius"
        except Exception as e:
            pass
        
        self.read_text(result)

    def cant_help(self):
        self.read_text("Lo siento, pero no puedo ayudarte con eso.")
        
    def change_name(self, new_name):
        """Changes the name of the assistant.
        
        Args:
            new_name (str): The new name of the assistant.
        """

        if new_name.startswith('cambiar nombre') or len(new_name) == 0:
            self.read_text("No has dicho ningun nombre. Tienes que decir cambiar nombre y un nombre")
        elif self.name.lower() == new_name:
            self.read_text("Ya me llamaba de antes " + self.name)
        else:
            self.name = new_name
            self.read_text("A partir de ahora me llamo " + self.name)

    def assistant_name(self):
        """Reads the name of the assistant and tells how to change the name"""

        self.read_text("Mi nombre es " + self.name + ". Puedes cambiar mi nombre con el comando cambiar nombre")

    def whats_my_age(self):
        """Tells the age of the person that the robot is actually recognizing.

        Note:
            This method is not done. Actually why don't have a way to aproximate a person's age
            we should try to explore if we can do it with FaceNet embeddings.
        """

        people_on_screen = []
        for person in self.actual_people:
            if time.time() - self.actual_people[person] < 1:
                people_on_screen.append(person)

        if len(people_on_screen) == 0:
            self.read_text("No veo a nadie, te estoy buscando...")
        elif len(people_on_screen) == 1:
            self.read_text(people_on_screen[0] + ", aún no soy capaz de aproximar la edad")
        else:
            people_string = people_on_screen[0]
            for i in range(1, len(people_on_screen)):
                people_string = people_string + (" y " if i >= len(people_on_screen) - 1 else ", ") + people_on_screen[i]
            self.read_text("Lo siento " + people_string + ", aún no sé hacer eso")

    def actual_hour(self):
        """Tells the local hour."""

        hora = datetime.now().strftime('%I:%M %p')
        self.read_text("Son las " + hora)

    def nothing(self):
        """Literally nothing, just for playing the deactivation sound."""

        play(TIME_OUT_SOUND)

    def who_am_i(self):
        """Tells the name of the people that is actually on screen.

        It uses the information that the recognizer provides.
        """
        people_on_screen = []
        for person in self.actual_people:
            if time.time() - self.actual_people[person] < 1:
                people_on_screen.append(person)

        if len(people_on_screen) == 0:
            self.read_text("No veo a nadie, te estoy buscando...")
        elif len(people_on_screen) == 1:
            self.read_text("Eres " + people_on_screen[0])
        else:
            people_string = people_on_screen[0]
            for i in range(1, len(people_on_screen)):
                people_string = people_string + (" y a " if i >= len(people_on_screen) - 1 else ", ") + people_on_screen[i]
            self.read_text("Veo a " + people_string)

    def repeat_text(self, text):
        """Repeats the speech that the user has prompted."""

        if text.startswith('repite') or len(text) == 0:
            self.read_text("No has dicho ninguna frase. Tienes que decir repite y una frase")
        else:
            self.read_text(text)

    def reproduce_video_youtube(self, video):
        """Opens the browser with the video that the user has prompted."""

        try:
            url = pywhatkit.playonyt(video)
            req = requests.get(url)
            soup = BeautifulSoup(req.text, features="html.parser")
            link = soup.find_all(name="title")[0]
            video_title = link.text.replace(' - YouTube', ' en Youtube')
            self.read_text("Reproduciendo " + video_title)
        except Exception:
            self.read_text("No he encontrado ningun video con ese nombre")

    def exit_assistant(self):
        """Exits from the assistant and from the recognizer if it is activated."""

        self.read_text("Me despido de ustedes, hasta la proxima.")
        self.read_text("Es coña, no puedo apagarme x d")
        #robot.exit = 1

    def unknown_command(self):
        """If the command that the user prompted is not recognized, this will execute."""

        self.read_text("Lo siento, no te he entendido.")


def main(args=None):
    rclpy.init(args=args)

    available_commands = [
    "buscar", "traducir a inglés", "tiempo actual", "cambiar nombre",
    "nombre del asistente", "edad actual", "hora actual", "no puedo ayudarte",
    "quién soy", "repetir texto", "reproducir video", "salir",
    "comando desconocido"
    ]   
    assistant = Asssistant("Sancho",available_commands)
    

    while rclpy.ok():
        if not assistant.text_queue.empty():
            msg = assistant.text_queue.get()
            text = msg.data

            assistant.process_text(text)

        rclpy.spin_once(assistant)

    rclpy.shutdown()