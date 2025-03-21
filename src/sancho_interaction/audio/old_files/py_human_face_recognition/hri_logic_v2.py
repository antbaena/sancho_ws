import time
import random
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from interbotix_xs_msgs.msg import JointGroupCommand
from std_msgs.msg import String

from .hri_bridge import HRIBridge
from .api.gui import mark_face
from human_face_recognition_msgs.srv import Detection, Recognition, Training, AudioLocation, GetString
from .camera_parameters import RESOLUTION, get_parameters

class HRILogicV2Node(Node):

    def __init__(self, hri_logic):
        super().__init__("hri_logic_v2")

        self.br = HRIBridge()
        self.hri_logic = hri_logic

        self.last_frame_msg = None
        self.subscription_camera = self.create_subscription(Image, "camera/color/image_raw", self.frame_callback, 1)
        self.publisher_recognition = self.create_publisher(Image, "camera/color/recognition", 1)
        self.sub_detection_data = self.create_subscription(JointState, "/wxxms/joint_states", self.head_position_callback, 10)
        self.head_movement = self.create_publisher(JointGroupCommand, "/head/movement", 1)
        
        self.audio_location_client = self.create_client(AudioLocation, "audio_location")
        while not self.audio_location_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('AudioLocation service not available, waiting again...')
        
        self.detection_client = self.create_client(Detection, "detection")
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detection service not available, waiting again...")

        self.recognition_client = self.create_client(Recognition, "recognition")
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Recognition service not available, waiting again...")

        self.training_client = self.create_client(Training, "recognition/training")
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Training service not available, waiting again...")

        self.get_people_client = self.create_client(GetString, "recognition/get_people")
        while not self.get_people_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Get People service not available, waiting again...")
        
    def frame_callback(self, frame_msg):
        self.last_frame_msg = frame_msg

    def head_position_callback(self, msg):
        self.hri_logic.pan = msg.position[0]
        self.hri_logic.tilt = msg.position[1]
class HRILogicV2:

    LOWER_BOUND = 0.75
    MIDDLE_BOUND = 0.80
    UPPER_BOUND = 0.90

    def __init__(self, ask_unknowns=True, draw_rectangle=True, show_distance=True, show_score=True):       
        self.ask_unknowns = ask_unknowns
        self.draw_rectangle = draw_rectangle
        self.show_distance = show_distance
        self.show_score = show_score
        self.last_voice = 0
        self.voice_thr = 0.5
        self.last_voice_time= time.time()
        self.pan = 0
        self.tilt = 0
      
        self.angle_threshold = 0.25
        self.max_time_without_interaction = 7
        
        self.last_interaction = 0
        self.interlocutor = None

        self.camera_matrix, self.camera_matrix_inv, self.dist_coeffs = get_parameters(RESOLUTION)

        self.node = HRILogicV2Node(self)
        
    def spin(self):
        while rclpy.ok():
            voice_intensity, human_voice_pan = self.process_audio_location(audio_threshold=self.voice_thr)
            #voice_intensity, human_voice_pan = self.audio_location_request()
            data = [] # Lista de [id, box]
            if self.node.last_frame_msg is not None:
                data = self.process_frame(self.node.last_frame_msg)
            
            if self.interlocutor is None: # Modo no-interaccion
                if voice_intensity != -1: # Si hay voz humana
                    self.node.head_movement.publish(JointGroupCommand(name="turret", cmd=[human_voice_pan, 0]))
                    self.node.get_logger().info(f"Moviendo a pan: {human_voice_pan*90:.3f}")
                    
                    i = 0
                    while i < len(data) and self.interlocutor is None:
                        id, box = data[i]
                        face_pan, face_tilt = self.get_angles_from_box(box)

                        if abs(human_voice_pan - (self.pan + face_pan)) < self.angle_threshold:  # Si hay una cara en el ángulo de incidencia del sonido
                            self.interlocutor = id  # Tenemos nuevo interlocutor y nos movemos hacia él
                            self.last_interaction = time.time() # Guardamos su interaccion

                            self.node.get_logger().info(f"Interactuando con {id}")
                            
                            self.node.head_movement.publish(JointGroupCommand(name="turret", cmd=[self.pan + face_pan, self.tilt + face_tilt]))
                        i += 1
                        
            else: # Modo interacción
                interlocutor_data = next(((id, box) for id, box in data if id == self.interlocutor), None)

                if interlocutor_data: # Si el interlocutor está en el rango de visión
                    id, box = interlocutor_data

                    face_pan, face_tilt = self.get_angles_from_box(box) # Nos movemos hacia él
                    self.node.head_movement.publish(JointGroupCommand(name="turret", cmd=[self.pan + face_pan, self.tilt + face_tilt]))
                    self.node.get_logger().info(f"Moviendo a la cara del interlocutor ({id})")

                    if abs(human_voice_pan - (self.pan + face_pan)) < self.angle_threshold and voice_intensity != -1: # Si hay voz donde está el interlocutor
                        self.last_interaction = time.time() # Reiniciamos el timeout por seguir interactuando
                        self.node.get_logger().info(f"El interlocutor ({id}) sigue interactuando ({human_voice_pan:.3f})")
                    else:
                        self.node.get_logger().info(f"Veo al interlocutor pero no está interactuando... (Está en {(self.pan + face_pan):.3f} y escucho voz en {human_voice_pan:.3f})")
                else:
                    self.node.get_logger().info("No veo al interlocutor en mi campo de visión")

                if time.time() - self.last_interaction > self.max_time_without_interaction: # Si hace timeout
                    self.interlocutor = None # Dejamos de interactuar con él
                    self.node.get_logger().info("Dejando de interactuar por timeout. Voy a [0, 0]")
                    self.node.head_movement.publish(JointGroupCommand(name="turret", cmd=[0, 0]))

            rclpy.spin_once(self.node)

    def process_audio_location(self, audio_threshold, time_threshold =1):
        """
        Process the audio location request and update the voice pan position.
        
        This function checks the intensity and position of the human voice,
        and updates the `last_voice` and `last_voice_time` attributes based on
        the current conditions and a specified threshold.

        Returns:
            tuple: A tuple containing the voice intensity and the updated human voice pan position.
        """
        voice_intensity, human_voice_pan = self.audio_location_request()

        # Check if the change in voice pan exceeds the threshold and if enough time has passed since the last update
        if abs(human_voice_pan - self.last_voice) > audio_threshold and (time.time() - self.last_voice_time) < time_threshold:
            # If conditions are met, reset the human voice pan to the last recorded voice pan
            self.node.get_logger().info("Mucha variacion de distancia me quedo con la anterior")
            human_voice_pan = self.last_voice
        else:
            # Otherwise, update the last voice pan and time with the current values
            self.last_voice = human_voice_pan
            self.last_voice_time = time.time()
        
        return voice_intensity, human_voice_pan

   
    def get_angles_from_box(self, box):
        (x, y, w, h) = box
        head_position = [x + (w / 2), y + (h / 2)]

        head_position_homogeneous = np.array([head_position[0], head_position[1], 1])
        head_position_camera = self.camera_matrix_inv @ head_position_homogeneous

        relative_pan_angle = -np.arctan(head_position_camera[0])
        relative_tilt_angle = np.arctan(head_position_camera[1])
        
        return relative_pan_angle, relative_tilt_angle

    def process_frame(self, frame_msg):
        data = [] # ADDED
        
        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")

        positions_msg, scores_msg = self.detection_request(frame_msg)  # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
                self.recognition_request(frame_msg, positions_msg[i]))  # Recogniion
            face_aligned, features, classified, distance, pos = (
                self.node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg))

            if distance < self.LOWER_BOUND: # No reconoce
                classified = None

                if scores[i] >= 1 and self.ask_unknowns:  # Si la imagen es buena
                    people_json_msg = self.get_people_request()
                    people = json.loads(people_json_msg.data)

                    # PARA CUANDO SE INTREGUE CON LO ANTERIOR, SI EMPIEZA POR 0x ES RANDOM
                    # SI NO ES PERSONA APRENDIDA
                    while classified is None or classified in people: 
                        classified = "0x" + random.randbytes(3).hex() # Random ID
                    
                    output = self.training_request(String(data="add_class"),String(data=classified), features_msg, pos_msg)
                    if output < 0:
                        self.node.get_logger().info(">> ERROR: Al añadir una clase")
                    else:
                        self.node.get_logger().info("Nueva persona aprendida: " + str(classified))
                        
            elif distance < self.UPPER_BOUND: # Reconoce con duda
                if scores[i] >= 1 and self.ask_unknowns:  # Si la imagen es buena
                    output = self.training_request(String(data="add_features"), String(data=classified), features_msg, pos_msg)
                    if output < 0:
                        self.node.get_logger().info(">> ERROR: Al añadir características a una clase")
                        
            else: # Reconoce bien
                output = self.training_request(String(data="refine_class"), String(data=classified), features_msg, pos_msg)
                if output < 0:
                    self.node.get_logger().info(">> ERROR: Al refinar una clase")
                
                data.append([classified, positions[i]]) # ADDED

            inter_time = self.max_time_without_interaction - (time.time() - self.last_interaction)
            mark_face(frame, positions[i], distance, self.MIDDLE_BOUND, self.UPPER_BOUND, classified=classified, drawRectangle=self.draw_rectangle,
                score=scores[i], showDistance=self.show_distance, showScore=self.show_score, 
                interlocutor=self.interlocutor, inter_time=0 if inter_time < 0 else inter_time)

        self.node.publisher_recognition.publish(self.node.br.cv2_to_imgmsg(frame, "bgr8"))
        
        return data
    
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

    def training_request(self, cmd_type_msg, classified_msg, features_msg, pos_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.classified = classified_msg
        training_request.features = features_msg
        training_request.pos = pos_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result

    def get_people_request(self):
        get_people_request = GetString.Request()

        future_get_people = self.node.get_people_client.call_async(get_people_request)
        rclpy.spin_until_future_complete(self.node, future_get_people)
        result_get_people = future_get_people.result()

        return result_get_people.text


def main(args=None):
    rclpy.init(args=args)

    hri_logic = HRILogicV2()
    hri_logic.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
