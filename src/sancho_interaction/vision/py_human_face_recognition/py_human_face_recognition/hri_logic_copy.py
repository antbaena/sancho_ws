import time
import random
import json
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from human_face_recognition_msgs.msg import BodyOrientation, FaceGuiRequest, HeadOrientation
from human_face_recognition_msgs.srv import Detection, Recognition, Training, AudioLocation, GetString, FaceGuiResponse, GetCameraParameters
from .hri_bridge import HRIBridge
from std_srvs.srv import Trigger
from .api.gui import mark_face

class HRILogicNode(Node):
    def __init__(self, hri_logic):
        super().__init__("hri_logic_v2")
        self.br = HRIBridge()
        self.last_frame_msg = None
        self.hri_logic = hri_logic
        self.setup_subscribers_publishers()
        self.setup_services()
        self.wait_for_services()
        self.get_camera_parameters()
        self.get_logger().info(f'HRI logic initialized!')

    def setup_subscribers_publishers(self):
        self.subscription_camera = self.create_subscription(Image, "camera/color/image_raw", self.frame_callback, 1)
        self.sub_detection_data = self.create_subscription(JointState, "/wxxms/joint_states", self.head_position_callback, 10)
        self.publisher_recognition = self.create_publisher(Image, "camera/color/recognition", 1)
        self.head_movement = self.create_publisher(HeadOrientation, "/head/movement", 1)
        self.input_tts = self.create_publisher(String, "input_tts", 1)
        self.publisher_face_name = self.create_publisher(FaceGuiRequest, "cara", 10)
        self.publisher_people = self.create_publisher(String, "robot/info/actual_people", 1)
        self.body = self.create_publisher(Twist, "/cmd_vel", 1)
        self.robot_orientation = self.create_publisher(BodyOrientation, "/robot/orientation", 1)
        self.gui_timeout_pub = self.create_publisher(String, "/gui/timeout", 1)
        

    def setup_services(self):
        self.ask_if_name_service = self.create_service(FaceGuiResponse, "peticion_yesno", self.ask_if_name)
        self.asking_service = self.create_service(FaceGuiResponse, "peticion_nombre", self.get_face_name)
        self.audio_stop_client = self.create_client(Trigger, 'stop_audio')

    def wait_for_services(self):
        self.audio_location_client = self.create_client(AudioLocation, "audio_location")
        self.detection_client = self.create_client(Detection, "detection")
        self.recognition_client = self.create_client(Recognition, "recognition")
        self.training_client = self.create_client(Training, "recognition/training")
        self.get_people_client = self.create_client(GetString, "recognition/get_people")
        self.get_camera_parameters_client = self.create_client(GetCameraParameters, 'get_camera_parameters')

        self._wait_for_service(self.audio_location_client, "AudioLocation")
        self._wait_for_service(self.detection_client, "Detection")
        self._wait_for_service(self.recognition_client, "Recognition")
        self._wait_for_service(self.training_client, "Training")
        self._wait_for_service(self.get_people_client, "Get People")
        self._wait_for_service(self.get_camera_parameters_client, 'GetCameraParameters')
        

    def _wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting again...')
            
    def get_camera_parameters(self):
        request = GetCameraParameters.Request()  # Crea la solicitud
        self.get_camera_parameters_client.call_async(request).add_done_callback(self.camera_parameters_callback)

    def camera_parameters_callback(self, future):
        response = future.result()
        if response:
            # Aquí almacenamos los parámetros de la cámara
            self.hri_logic.camera_matrix = np.array(response.camera_matrix).reshape(3, 3)
            self.hri_logic.camera_matrix_inv = np.array(response.camera_matrix_inv).reshape(3, 3)
            self.hri_logic.dist_coeffs = np.array(response.dist_coeffs)
            self.get_logger().info('Camera parameters successfully obtained.')
        else:
            self.get_logger().error('Failed to get camera parameters.')

    def frame_callback(self, frame_msg):
        self.last_frame_msg = frame_msg

    def head_position_callback(self, msg):
        self.hri_logic.pan, self.hri_logic.tilt = msg.position[:2]

    def get_face_name(self, request, response):
        self.hri_logic.face_name = request.texto.data.split()[0]
        response.result = 1
        return response

    def ask_if_name(self, request, response):
        self.hri_logic.yes_no_result = request.texto.data
        response.result = 1
        return response


class HRILogic:
    LOWER_BOUND = 0.75
    MIDDLE_BOUND = 0.80
    UPPER_BOUND = 0.90

    def __init__(self, ask_unknowns=True, draw_rectangle=True, show_distance=True, show_score=True):
        self.ask_unknowns = ask_unknowns
        self.draw_rectangle = draw_rectangle
        self.show_distance = show_distance
        self.show_score = show_score
        self.initialize_variables()
        self.camera_matrix, self.camera_matrix_inv, self.dist_coeffs = None, None, None
        self.node = HRILogicNode(self)

    def initialize_variables(self):
        self.last_voice = 0
        self.voice_thr = 1
        self.last_voice_time = time.time()
        self.pan = 0
        self.tilt = 0
        self.asking_name = False
        self.ask_if_name_mode = False
        self.ask_name_if_no = False
        self.yes_no_result = None
        self.face_name = None
        self.angle_threshold = 0.10
        self.max_time_without_interaction = 7
        self.last_interaction = time.time()
        self.interlocutor = None
        self.timer_ask_name = time.time()
        self.actual_people = {}
        self.last_idle_time = time.time()
        
        self.candidate_interlocutor = None
        self.last_seen_candidate_interlocutor  =time.time()
        self.first_seen_candidate_interlocutor = time.time()
        self.times_seen_candidate_interlocutor = 0
        self.is_verifying_candidate = False
        self.dalay_asking_name = 2
        self.greetings = ["Hola", "Qué tal", "Saludos", "Hey", "Hola, mucho gusto"]
        
    def spin(self):
        while rclpy.ok():

            voice_intensity, human_voice_pan = self.process_audio_location()
                     
            data = self.process_frame(self.node.last_frame_msg) if self.node.last_frame_msg else []
                      
            self.handle_interaction(voice_intensity, human_voice_pan, data)
        
            rclpy.spin_once(self.node)
        

    def process_audio_location(self, audio_threshold=None):
        audio_threshold = audio_threshold or self.voice_thr
        voice_intensity, human_voice_pan = self.audio_location_request()
        if abs(human_voice_pan - self.last_voice) > audio_threshold:
            human_voice_pan = self.last_voice
        else:
            self.last_voice = human_voice_pan
            self.last_voice_time = time.time()
        return voice_intensity, human_voice_pan

    def handle_interaction(self, voice_intensity, human_voice_pan, data):
        
        if self.interlocutor is None:   # Si hay voz detectada
                if not self.is_verifying_candidate and voice_intensity != -1:
                    print(f"buscando interlocutor")
                    self.search_for_interlocutor(human_voice_pan, data)
                elif(self.is_verifying_candidate):
                    print(f"verificando interlocutor")
                    # self.verify_candidate_interlocutor_talk(human_voice_pan, data, voice_intensity)
                    self.verify_candidate_interlocutor_seen(data)
                elif time.time() - self.last_interaction > 4:  # Si ha pasado suficiente tiempo sin interacción
                    self.idle()  # Mueve la cabeza de forma aleatoria
        else:
            self.interact_with_person(voice_intensity, human_voice_pan, data)

    def search_for_interlocutor(self, human_voice_pan, data): 
        found_target = False
        self.last_interaction = time.time()
        for id, box, score, _, distance, _, _ in data:
            if self.is_valid_face(human_voice_pan, box, id, score):
                face_pan, face_tilt = self.get_angles_from_box(box)
                self.move_head(face_pan, face_tilt)
                found_target = True
                self.candidate_interlocutor = id
                self.is_verifying_candidate = True
                self.first_seen_candidate_interlocutor = time.time()
                self.last_seen_candidate_interlocutor = time.time()
                self.times_seen_candidate_interlocutor = 1             
                break
        if not found_target:
            self.move_head(human_voice_pan, random.uniform(0.0, -0.3))
        

    def verify_candidate_interlocutor_talk(self, human_voice_pan, data, voice_intensity): 
        self.last_interaction = time.time()
        for id, box, score, _, distance, _, _ in data:
                if id == self.candidate_interlocutor:
                    face_pan, face_tilt = self.get_angles_from_box(box)
                    self.move_head(face_pan, face_tilt)
                    print(f"miau1")
                    if voice_intensity != -1 and self.is_valid_face(human_voice_pan, box, id, score):
                        print(f"miau2")
                        self.last_seen_candidate_interlocutor = time.time()
                        self.times_seen_candidate_interlocutor += 1
                        print(f"Acabo de ver al interlocutor {id} por audio por {self.times_seen_candidate_interlocutor} vez")
                        if self.last_seen_candidate_interlocutor - self.first_seen_candidate_interlocutor > 2 and self.times_seen_candidate_interlocutor > 5:                     
                            self.interlocutor = self.candidate_interlocutor
                            self.candidate_interlocutor = None
                            self.last_seen_candidate_interlocutor = None
                            self.first_seen_candidate_interlocutor = None
                            self.is_verifying_candidate = False
                            self.times_seen_candidate_interlocutor = 0
                            self.greet_or_ask_name(id, score, distance)
                            break
                # else:
                #     self.candidate_interlocutor = id
                #     self.first_seen_candidate_interlocutor = time.time()
                #     self.times_seen_candidate_interlocutor = 1
                
        if(self.last_seen_candidate_interlocutor is not None and time.time() - self.last_seen_candidate_interlocutor > 3):
            print(f"Hace mucho tiempo que no veo al interlocutor, a tomar por culo")
            self.interlocutor = None
            self.candidate_interlocutor = None
            self.last_seen_candidate_interlocutor = None
            self.is_verifying_candidate = False 
                
    def verify_candidate_interlocutor_seen(self, data): 
        self.last_interaction = time.time()
        print("verificando")
        for id, box, score, _, distance, _, _ in data:
                if id == self.candidate_interlocutor:
                    face_pan, face_tilt = self.get_angles_from_box(box)
                    self.move_head(face_pan, face_tilt)
                    self.last_seen_candidate_interlocutor = time.time()
                    self.times_seen_candidate_interlocutor += 1
                    if self.last_seen_candidate_interlocutor - self.first_seen_candidate_interlocutor > 2 and self.times_seen_candidate_interlocutor > 3:                     
                        self.interlocutor = self.candidate_interlocutor
                        self.candidate_interlocutor = None
                        self.last_seen_candidate_interlocutor = None
                        self.is_verifying_candidate = False
                        self.times_seen_candidate_interlocutor = 0
                        self.actual_people[self.face_name] = time.time()
                        self.greet_or_ask_name(id, score, distance)
                        
                
                
        if(self.last_seen_candidate_interlocutor is not None and time.time() - self.last_seen_candidate_interlocutor > 3):
            self.interlocutor = None
            self.candidate_interlocutor = None
            self.last_seen_candidate_interlocutor = None
            self.is_verifying_candidate = False         
               
    def move_head(self, pan, tilt):
        self.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), pan=pan, tilt=tilt))
        
    def idle(self):
        current_time = time.time()

        # Definir cuánto tiempo debe pasar antes de mover la cabeza de nuevo 
        wait_time = random.uniform(6, 10)

        # Solo mover la cabeza si ha pasado suficiente tiempo desde el último movimiento
        if current_time - self.last_idle_time >= wait_time:
            # Generar valores aleatorios para pan y tilt
            min_pan, max_pan = -0.4, 0.4
            min_tilt, max_tilt = -0.2, 0.2
            random_pan = random.uniform(min_pan, max_pan)
            random_tilt = random.uniform(min_tilt, max_tilt)

            # Publicar el movimiento de la cabeza con valores aleatorios
            self.node.head_movement.publish(HeadOrientation(
                mode=String(data="absolute"),
                pan=random_pan,
                tilt=random_tilt
            ))

            self.last_idle_time = time.time()    
        
    def is_valid_face(self, human_voice_pan, box, id, score):
        face_pan, face_tilt = self.get_angles_from_box(box)
        return abs(human_voice_pan - (self.pan + face_pan)) < self.angle_threshold

    def greet_or_ask_name(self, id, score, distance):
        stop_audio_request = Trigger.Request()
        future_stop_audio = self.node.audio_stop_client.call_async(stop_audio_request)
        print("miau1")
        rclpy.spin_until_future_complete(self.node, future_stop_audio)
        result_play_audio = future_stop_audio.result()
        self.node.get_logger().info(str(result_play_audio.message))
        print("miau2")
        if(result_play_audio.success is True):
            if id.startswith("0x") and score > 1 and self.ask_unknowns:
                self.read_text("Hola, ¿Cual es tu nombre?")
                #TODO Logica para bloquear el microfono
                self.asking_name = True
            elif self.ask_unknowns and self.LOWER_BOUND <= distance < self.MIDDLE_BOUND and score > 1:
                self.read_text(f"Um, no te reconozco con seguridad, ¿eres {id}?")
                self.ask_if_name_mode = True
            self.timer_ask_name = time.time()

    def interact_with_person(self, voice_intensity, human_voice_pan, data):
        interlocutor_data = next((d for d in data if d[0] == self.interlocutor), None)
        if interlocutor_data:
            id, box, score, face_aligned_msg, distance, features, _ = interlocutor_data
            if self.asking_name:
                self.handle_asking_name(face_aligned_msg, id, features)
            elif self.ask_if_name_mode:
                self.handle_ask_if_name(face_aligned_msg, id, features)
            elif self.ask_name_if_no:
                self.handle_name_if_no(face_aligned_msg, features)           
            self.move_towards_person(box)
            if(voice_intensity != -1 and self.is_valid_face(human_voice_pan, box, id, score)): 
                self.last_interaction = time.time()
        self.check_interaction_timeout()

    def handle_asking_name(self, face_aligned_msg, id, features):
        if time.time() - self.timer_ask_name > self.dalay_asking_name:
            if self.face_name is None:
                self.node.publisher_face_name.publish(FaceGuiRequest(mode=0, face=face_aligned_msg))
            else:
                 # Seleccionar un saludo aleatorio
                random_greeting = random.choice(self.greetings)
                
                # Formar el string con el saludo aleatorio y el nombre de la persona
                self.read_text(f"{random_greeting} {self.face_name}, encantado de conocerte!")
                
                self.rename_person(id, features)              

    def handle_ask_if_name(self, face_aligned_msg, id, features):
        if self.yes_no_result == "yes":
            self.node.publisher_face_name.publish(FaceGuiRequest(mode=2, face=face_aligned_msg, name=id))
        elif self.yes_no_result == "no" and time.time() - self.timer_ask_name > 2:
            self.read_text("¿Cual es tu nombre?")
            self.ask_name_if_no = True

    def handle_name_if_no(self, face_aligned_msg, features):
        if self.face_name:
            self.rename_person(None, features)
        else:
            self.node.publisher_face_name.publish(FaceGuiRequest(mode=0, face=face_aligned_msg))

    def rename_person(self, id, features):
        result, message = self.training_request(String(data="rename_class"), String(data=json.dumps({
                                "old_name": id,
                                "new_name": self.face_name
                            }))) # Renombramos la clase (al nombre de la persona)
        self.actual_people[id] = self.face_name
        self.reset_name_flags()

    def reset_name_flags(self):
        self.asking_name = False
        self.ask_if_name_mode = False
        self.ask_name_if_no = False
        self.face_name = None

    def move_towards_person(self, box):
        face_pan, face_tilt = self.get_angles_from_box(box)
        self.node.head_movement.publish(HeadOrientation(mode=String(data="relative"), pan=face_pan, tilt=face_tilt))

    def check_interaction_timeout(self):
        if time.time() - self.last_interaction > self.max_time_without_interaction:
            self.interlocutor = None
            
        #Si y solo si lo ha visto durante mas de x tiempo

    def audio_location_request(self):
        audio_req = AudioLocation.Request()
        audio_res = self.node.audio_location_client.call(audio_req)
        return audio_res.intensity, audio_res.pan

    def process_frame(self, frame_msg):
        data = []
    
        people = json.loads(self.get_people_request().data)
        
        deleted_classes = [person for person in self.actual_people if person not in people]
        for person in deleted_classes:
            del self.actual_people[person]
       
        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")
       
        positions_msg, scores_msg = self.detection_request(frame_msg)
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
       
        inter_time = max(0, self.max_time_without_interaction - (time.time() - self.last_interaction))
        
        for i, position in enumerate(positions):
            
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = self.recognition_request(frame_msg, positions_msg[i])
            face_aligned, features, classified, distance, pos = self.node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg)
            classified = self.handle_classification(classified, distance, features, scores[i], people, pos)
            if classified is not None:
                data.append([classified, position, scores[i], face_aligned_msg, distance, features, pos])

            mark_face(frame, position, distance, self.MIDDLE_BOUND, self.UPPER_BOUND, 
                    classified=classified, drawRectangle=self.draw_rectangle, 
                    score=scores[i], showDistance=self.show_distance, 
                    showScore=self.show_score, interlocutor=self.interlocutor, 
                    inter_time=inter_time)


        self.node.publisher_people.publish(String(data=json.dumps(self.actual_people)))
        self.node.publisher_recognition.publish(self.node.br.cv2_to_imgmsg(frame, "bgr8"))

        return data

    def handle_classification(self, classified, distance, features, score, people, position):
        if distance < self.LOWER_BOUND:
            if score >= 1 and self.ask_unknowns:
                classified = self.generate_unique_id(people)
                _, message = self.add_new_class(classified, features)
                self.node.get_logger().info(message.data)
        elif distance < self.UPPER_BOUND and classified and not classified.startswith("0x"):
            self.actual_people[classified] = time.time()
        elif classified and not classified.startswith("0x"):
            if classified not in self.actual_people or (time.time() - self.actual_people[classified]) > 30:
                self.read_text(f"Bienvenido de vuelta {classified}")
            self.actual_people[classified] = time.time()
            _, message = self.refine_class(classified, features, position)
            self.node.get_logger().info(message.data)

        return classified

    def generate_unique_id(self, people):
        while True:
            classified = "0x" + random.randbytes(3).hex()
            if classified not in people:
                return classified

    def add_new_class(self, classified, features):
        result, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                    "class_name": classified,
                    "features": features
                }))) # Añadimos nueva clase
        return result, message
    
    def refine_class(self, classified, features, position):
        result, message = self.training_request(String(data="refine_class"), String(data=json.dumps({
                "class_name": classified,
                "features": features,
                "position": position
            }))) # Refinamos la clase
        return result, message
        
    def get_angles_from_box(self, box):
        (x, y, w, h) = box
        head_position = [x + (w / 2), y + (h / 2)]

        head_position_homogeneous = np.array([head_position[0], head_position[1], 1])
        head_position_camera = self.camera_matrix_inv @ head_position_homogeneous

        relative_pan_angle = -np.arctan(head_position_camera[0])
        relative_tilt_angle = np.arctan(head_position_camera[1])
        
        return relative_pan_angle, relative_tilt_angle

    def read_text(self, text):
        msg = String()
        msg.data = text
        self.node.input_tts.publish(msg)
        
    def audio_location_request(self):
        audio_location_request = AudioLocation.Request()

        future_audio_location = self.node.audio_location_client.call_async(audio_location_request)
        rclpy.spin_until_future_complete(self.node, future_audio_location)
        result_audio_location = future_audio_location.result()

        return result_audio_location.intensity, result_audio_location.angle_rad

    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.node.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg):
        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg

        future_recognition = self.node.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self.node, future_recognition)
        result_recognition = future_recognition.result()

        return (
            result_recognition.face_aligned,
            result_recognition.features,
            result_recognition.classified,
            result_recognition.distance,
            result_recognition.pos,
        )

    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message

    def get_people_request(self):
        get_people_request = GetString.Request()

        future_get_people = self.node.get_people_client.call_async(get_people_request)
        rclpy.spin_until_future_complete(self.node, future_get_people)
        result_get_people = future_get_people.result()

        return result_get_people.text


def main(args=None):
    rclpy.init(args=args)

    hri_logic = HRILogic()
    hri_logic.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
