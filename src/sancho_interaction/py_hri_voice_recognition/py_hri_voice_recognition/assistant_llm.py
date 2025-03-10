import datetime
import time
import requests
import json

import pywhatkit
import wikipedia
from bs4 import BeautifulSoup
from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .api.sound import play
from .api.sounds import ACTIVATION_SOUND, TIME_OUT_SOUND

from human_face_recognition_msgs.srv import HelperMode, GetString


class Asssistant(Node):

    def __init__(self, name, recognizer):
        super().__init__("assistant")
        
        self.name = name
        #self.accent = ACCENT_SPAIN
        self.said_name = False
        self.recognizer = recognizer

        self.text_queue = Queue(maxsize=10)

        self.subscription = self.create_subscription(String, 'audio/assistant/text', self.text_callback, 10)
        self.helper_mode_client = self.create_client(HelperMode, 'audio/helper/mode')
        while not self.helper_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Helper mode service not available, waiting again...')

        self.get_actual_people_client = self.create_client(GetString, 'video/get/actual_people') # We dont want to block if this service is not available
        self.input_tts = self.create_publisher(String, 'input_tts', 10)
    
    def read_text(self, text):
        #msg = [text, self.accent]
        #msg_json = json.dumps(msg)
        
        self.input_tts.publish(String(data=text))
    
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
    
    def get_actual_people_request(self):
        result = "{}"

        if self.get_actual_people_client.service_is_ready():
            get_actual_people_request = GetString.Request()
            
            future_actual_people = self.get_actual_people_client.call_async(get_actual_people_request)
            rclpy.spin_until_future_complete(self, future_actual_people)
            result_actual_people = future_actual_people.result()

            result = result_actual_people.text.data

        return result

    def process_input(self, text):
        """Process the user's speech and determine if the assistant should be activated.

        The function checks whether the user has mentioned the assistant's name and if the assistant has been
        previously activated. If the user has not called the assistant yet, the function ignores the input.

        Args:
            text (str): The speech input from the user.
        """

        print(">> USUARIO: " + text)
        text_lower = text.lower()

        if (self.name.lower() in text_lower and not self.said_name) or (self.name.lower() == text_lower and self.said_name):
            self.said_name = True
            self.helper_mode_request(1) # Listen for command
            play(ACTIVATION_SOUND)
        elif self.said_name:
            self.said_name = False
            self.helper_mode_request(0) # Listen for name
            self.process_text(text_lower)
            
    def process_text(self, text):
        most_similar_command = self.recognizer.recognize_command(text)
        self.process_command(most_similar_command, text)
        
    def process_command(self, command, original_text):
        """Process the user's speech as a command.

        Note:
            If adding a new command, please, follow the same style, do not add the command
            function in this method, create a new private function and delegate the behaviour there.

        Args:
            command (str): The command as a string.
        """
        self.get_logger().info(command)
        if 'busca' in command:  # Busca en Wikipedia lo que le digas
            topic = original_text.replace('busca', '')
            self.search(topic)
        #elif 'cambiar acento' in command:  # Cambia el acento del asistente
        #    accent_text = original_text.replace('cambiar acento ', '')
        #    self.change_accent(accent_text)
        elif 'cambiar nombre' in command:  # Cambia el nombre del asistente
            new_name = original_text.replace('cambiar nombre ', '')
            self.change_name(new_name)
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
        elif 'repite' in command:  # Repite la frase que digas
            text = original_text
            self.repeat_text(text)
        elif 'reproduce' in command:  # Reproduce una canción en YouTube
            video = original_text.replace('reproduce', '')
            self.reproduce_video_youtube(video)
        elif 'salir' == command:  # Finaliza la ejecución
            self.exit_assistant()
        else:  # En caso de no reconocer ningún comando
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

    # def change_accent(self, accent_text):
    #     """Changes the accent of the text to speech voice.

    #     Args:
    #         accent_text (str): The accent in a string way in order to be processed.
    #     """

    #     new_accent = None
    #     if "mexicano" == accent_text:
    #         new_accent = ACCENT_MEXICO
    #     elif "español" == accent_text:
    #         new_accent = ACCENT_SPAIN

    #     if new_accent is None:
    #         self.read_text("Las opciones para cambiar el acento son: mexicano y español")
    #     elif self.accent == new_accent:
    #         self.read_text("Ya tenia de antes el acento " + accent_text)
    #     else:
    #         self.accent = new_accent
    #         self.read_text("A partir de ahora tendré el acento " + accent_text)

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

        actual_people_json = self.get_actual_people_request()
        actual_people = json.loads(actual_people_json)

        people_on_screen = []
        for person in actual_people:
            if time.time() - actual_people[person] < 1:
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

        hora = datetime.datetime.now().strftime('%I:%M %p')
        self.read_text("Son las " + hora)

    def nothing(self):
        """Literally nothing, just for playing the deactivation sound."""

        play(TIME_OUT_SOUND)

    def who_am_i(self):
        """Tells the name of the people that is actually on screen.

        It uses the information that the recognizer provides.
        """

        actual_people_json = self.get_actual_people_request()
        actual_people = json.loads(actual_people_json)

        people_on_screen = []
        for person in actual_people:
            if time.time() - actual_people[person] < 1:
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

        self.read_text("Saliendo...")
        #robot.exit = 1

    def unknown_command(self):
        """If the command that the user prompted is not recognized, this will execute."""

        self.read_text("Lo siento, no te he entendido.")


def main(args=None):
    rclpy.init(args=args)
    commands = [
        "busca", 
        "cambiar acento", 
        "cambiar nombre", 
        "como te llamas", 
        "edad", 
        "hora", 
        "nada", 
        "quien soy", 
        "repite", 
        "reproduce", 
        "salir"
    ]

    recognizer = CommandRecognizer(recognizable_commands=commands)
    assistant = Asssistant("Alexa", recognizer)

    while(rclpy.ok()):
        if not assistant.text_queue.empty():
            msg = assistant.text_queue.get()
            text = msg.data

            assistant.process_input(text)

        rclpy.spin_once(assistant)

    rclpy.shutdown()
    
    
from transformers import AutoTokenizer, AutoModel
import torch
import torch.nn.functional as F

class CommandRecognizer:
    def __init__(self,recognizable_commands=None, model_name='hiiamsid/sentence_similarity_spanish_es'):
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModel.from_pretrained(model_name)
        self.preestablished_commands = recognizable_commands
        self.command_embeddings = self.compute_embeddings(self.preestablished_commands)

    def mean_pooling(self, model_output, attention_mask):
        token_embeddings = model_output[0] # First element of model_output contains all token embeddings
        input_mask_expanded = attention_mask.unsqueeze(-1).expand(token_embeddings.size()).float()
        return torch.sum(token_embeddings * input_mask_expanded, 1) / torch.clamp(input_mask_expanded.sum(1), min=1e-9)

    def normalize_embeddings(self, embeddings):
        return F.normalize(embeddings, p=2, dim=1)

    def compute_embeddings(self, sentences):
        encoded_input = self.tokenizer(sentences, padding=True, truncation=True, return_tensors='pt')
        with torch.no_grad():
            model_output = self.model(**encoded_input)
        sentence_embeddings = self.mean_pooling(model_output, encoded_input['attention_mask'])
        return self.normalize_embeddings(sentence_embeddings)

    def recognize_command(self, user_input):
        user_embeddings = self.compute_embeddings([user_input])
        similarities = []
        for command_embedding in self.command_embeddings:
            similarity = torch.cosine_similarity(user_embeddings, command_embedding)
            similarities.append(similarity.item())
        #print(max(similarities))
        most_similar_command_index = similarities.index(max(similarities))
        most_similar_command = self.preestablished_commands[most_similar_command_index]
        return most_similar_command

