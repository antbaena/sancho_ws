import time
import json
from queue import Queue
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .hri_bridge import HRIBridge
from .api.gui import get_name, ask_if_name, mark_face
from human_face_recognition_msgs.srv import (
    Detection,
    Recognition,
    Training,
    GetString,
    FaceGuiResponse,
    Tracking,
)
from human_face_recognition_msgs.msg import FacePosition, FaceGuiRequest


class HRILogicNode(Node):

    def __init__(self, hri_logic):
        """Initializes the logic node. Subscribes to camera and publishes recognition images"""

        super().__init__("hri_logic")

        self.br = HRIBridge()
        self.hri_logic = hri_logic

        self.data_queue = Queue(
            maxsize=1
        )  # Queremos que olvide frames antiguos, siempre a por los mas nuevos
        self.get_actual_people_service = self.create_service(
            GetString, "video/get/actual_people", self.get_actual_people
        )
        self.subscription_camera = self.create_subscription(
            Image, "camera/color/image_raw", self.frame_callback, 1
        )
        self.publisher_recognition = self.create_publisher(
            Image, "camera/color/recognition", 1
        )
        self.publisher_people = self.create_publisher(
            String, "robot/info/actual_people", 1
        )
        self.detection_client = self.create_client(Detection, "detection")

        self.publisher_tracking = self.create_publisher(FacePosition, "tracking", 10)
        self.tracking_client = self.create_client(Tracking, "track")
        self.publisher_face_name = self.create_publisher(FaceGuiRequest, "cara", 10)
        self.ask_if_name = self.create_service(
            FaceGuiResponse, "peticion_yesno", self.ask_if_name
        )
        self.asking_service = self.create_service(
            FaceGuiResponse, "peticion_nombre", self.get_face_name
        )

        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detection service not available, waiting again...")

        self.recognition_client = self.create_client(Recognition, "recognition")
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Recognition service not available, waiting again..."
            )

        self.training_client = self.create_client(Training, "recognition/training")
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Training service not available, waiting again...")

        self.input_tts = self.create_publisher(String, "input_tts", 10)

    def ask_if_name(self, request, response):
        self.hri_logic.asking_confirmation = False
        self.hri_logic.yes_no_result = str(request.texto.data)
        response.result = 1
        return response

    def get_face_name(self, request, response):
        self.hri_logic.asking_name = False
        self.hri_logic.face_name = str(request.texto.data)
        response.result = 1
        print("Nuevo nombre: " + self.hri_logic.face_name)
        return response

    def get_actual_people(self, request, response):
        actual_people_json = json.dumps(self.hri_logic.actual_people)

        response.text = String(data=actual_people_json)

        return response

    def frame_callback(self, frame_msg):
        """Callback for camera frames"""

        if self.data_queue.empty():  # We don't want blocking
            self.data_queue.put(frame_msg)


class HRILogic:

    LOWER_BOUND = 0.75
    MIDDLE_BOUND = 0.80
    UPPER_BOUND = 0.90

    def __init__(
        self,
        ask_unknowns=True,
        draw_rectangle=True,
        show_distance=True,
        show_score=True,
    ):
        """Initializes logic that uses the logic node. This class has all the logic related to how to train the recognizer,
        when to detect faces, when we will ask a new person for his name, which values we consider reliable etc...

        Args:
            ask_unknowns (bool): If true will ask for new people. If false will only recognize already known people.
            draw_rectangle (bool): If true will draw a rectangle around the detected face on the frame published on recognition topic.
            show_distance (bool): If true will draw recognition score below the rectangle.
            show_score (bool):  If true will draw detector score below the distance.
        """
        self.ask_name_mode = False
        self.tracking = False
        self.asking_name = False
        self.face_name = ""
        self.last_tracking_face_timer = time.time()
        
        self.ask_if_mame_mode =False
        self.asking_confirmation = False
        self.if_no = False
        self.yes_no_result = ""
        self.tracking_face = ""

        self.ask_unknowns = ask_unknowns
        self.draw_rectangle = draw_rectangle
        self.show_distance = show_distance
        self.show_score = show_score

        self.actual_people = {}
        self.node = HRILogicNode(self)

    def read_text(self, text):
        """Reads text with speech to text

        Args:
            text (str): Text to be readed.
        """

        self.node.input_tts.publish(String(data=text))

    def spin(self):
        """Spins the logic node searching for new frames. If one is detected, process the frame."""

        while rclpy.ok():
            if not self.node.data_queue.empty():
                frame_msg = self.node.data_queue.get()
                self.nuevaLogica(frame_msg)
                # self.process_frame(frame_msg)
                # cv2.waitKey(1)

            rclpy.spin_once(self.node)

    def process_frame(self, frame_msg):
        """Performs all the logic. Process the frame detecting and recognizing faces on the frame.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format
        """

        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")

        positions_msg, scores_msg = self.detection_request(frame_msg)  # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
                self.recognition_request(frame_msg, positions_msg[i])
            )  # Recogniion
            face_aligned, features, classified, distance, pos = (
                self.node.br.msg_to_recognizer(
                    face_aligned_msg,
                    features_msg,
                    classified_msg,
                    distance_msg,
                    pos_msg,
                )
            )

            if distance < self.LOWER_BOUND:  # No sabe quien es, pregunta por el nombre
                classified = None
                # Vamos a hacer un asignacion dummy para que no sea mongolico

                if (
                    scores[i] >= 1 and self.ask_unknowns
                ):  # Si la imagen es buena, pregunta por el nombre, para que no coja una imagen mala
                    self.read_text("¿Cual es tu nombre?")
                    classified = get_name(
                        face_aligned
                    )  # Poner aqui una lista o si no que meta un nuevo a mano, pero por si ya es que seleccione
                    if classified is not None:
                        self.actual_people[classified] = time.time()
                        already_known = self.training_request(
                            String(data="add_class"),
                            String(data=classified),
                            features_msg,
                            pos_msg,
                        )
                        distance = 1

                        if already_known == 1:
                            self.read_text(
                                "Perdona "
                                + classified
                                + ", no te había reconocido bien"
                            )
                        elif already_known == 0:
                            self.read_text(
                                "Bienvenido " + classified + ", no te conocía"
                            )
                        else:
                            self.node.get_logger().info(
                                ">> ERROR: Algo salio mal al agregar una nueva clase"
                            )
            elif distance < self.MIDDLE_BOUND:  # Cree que es alguien, pide confirmacion
                if scores[i] > 1 and self.ask_unknowns:  # Pero solo si la foto es buena
                    classified_tried = classified
                    self.read_text(
                        "Creo que eres " + classified_tried + ", ¿es cierto?"
                    )
                    answer = self.confirmation_request(face_aligned)
                    # answer = ask_if_name(face_aligned, classified_tried)
                    if answer:  # Si dice que si es esa persona
                        self.actual_people[classified] = time.time()
                        output = self.training_request(
                            String(data="add_features"),
                            String(data=classified),
                            features_msg,
                            pos_msg,
                        )
                        if output >= 0:
                            self.read_text(
                                "Gracias "
                                + classified
                                + ", me gusta confirmar que estoy reconociendo bien"
                            )
                        else:
                            self.node.get_logger().info(
                                ">> ERROR: Algo salio mal al agregar features a una clase"
                            )
                    else:  # Si dice que no, le pregunta el nombre
                        self.read_text("Entonces, ¿Cual es tu nombre?")
                        classified = get_name(face_aligned)
                        if classified is not None:
                            self.actual_people[classified] = time.time()
                            already_known = self.training_request(
                                String(data="add_class"),
                                String(data=classified),
                                features_msg,
                                pos_msg,
                            )
                            distance = 1

                            if classified_tried == classified:
                                self.read_text(
                                    classified + ", no me marees, por favor."
                                )
                            elif already_known == 1:
                                self.read_text(
                                    "Perdona " + classified + ", te he confundido."
                                )
                            elif already_known == 0:
                                self.read_text(
                                    "Encantando de conocerte "
                                    + classified
                                    + ", perdona por confundirte"
                                )
                            else:
                                self.node.get_logger().info(
                                    ">> ERROR: Algo salio mal al agregar una nueva clase"
                                )
            elif (
                distance < self.UPPER_BOUND
            ):  # Sabe que es alguien pero lo detecta un poco raro
                self.actual_people[classified] = time.time()
                # classifier.addFeatures(classified, features)
            else:  # Reconoce perfectamente
                if (
                    classified not in self.actual_people
                    or (time.time() - self.actual_people[classified]) > 30
                ):
                    self.read_text("Bienvenido de vuelta " + classified)

                self.actual_people[classified] = time.time()
                output = self.training_request(
                    String(data="refine_class"),
                    String(data=classified),
                    features_msg,
                    pos_msg,
                )
                if output < 0:
                    self.node.get_logger().info(">> ERROR: Al refinar una clase")

            mark_face(
                frame,
                positions[i],
                distance,
                self.MIDDLE_BOUND,
                self.UPPER_BOUND,
                classified=classified,
                drawRectangle=self.draw_rectangle,
                score=scores[i],
                showDistance=self.show_distance,
                showScore=self.show_score,
            )

        actual_people_json = json.dumps(self.actual_people)
        self.node.publisher_people.publish(String(data=actual_people_json))
        self.node.publisher_recognition.publish(
            self.node.br.cv2_to_imgmsg(frame, "bgr8")
        )

    def nuevaLogica(self, frame_msg):
        """Performs all the logic. Process the frame detecting and recognizing faces on the frame.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format
        """

        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")

        positions_msg, scores_msg = self.detection_request(frame_msg)  # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        
        #Itero sobre todas las caras que veo en un frame
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
                self.recognition_request(frame_msg, positions_msg[i])
            )  # Recogniion
            face_aligned, features, classified, distance, pos = (
                self.node.br.msg_to_recognizer(
                    face_aligned_msg,
                    features_msg,
                    classified_msg,
                    distance_msg,
                    pos_msg,
                )
            )
           
            #No esta en ningun modo especifico
            if not self.tracking and not self.asking_confirmation:
                if (
                    distance < self.LOWER_BOUND
                ):  # No sabe quien es, pregunta por el nombre

                    # Vamos a hacer un asignacion dummy para que no sea mongolico
                    # res = self.tracking_request()
                    self.tracking = True
                    self.asking_name = True
                    print("solo tiene q decirlo una vez")
                    self.read_text(
                        "¿Cual es tu nombre?"
                    )
                    classified = "dummy"
                    self.actual_people[classified] = time.time()
                    already_known = self.training_request(
                        String(data="add_class"),
                        String(data=classified),
                        features_msg,
                        pos_msg,
                    )
                    distance = 1
                    # Aqui ya conoce al dummy
                    self.last_tracking_face_timer = time.time()
                    self.track_face(positions[i])
                elif (
                    distance < self.MIDDLE_BOUND            
                ):  # Cree que es alguien, pide confirmacion
                    
                    if (
                        scores[i] > 0 and self.ask_unknowns
                    ):  # Pero solo si la foto es buena
                        print("te veo bakanisimo")
                        self.asking_confirmation = True
                        self.tracking = True
                        self.tracking_face = classified

                        self.last_tracking_face_timer = time.time()
                        self.track_face(positions[i])
                    else:
                        print("te veo regulin")
                elif (
                    distance < self.UPPER_BOUND
                ):  # Sabe que es alguien pero lo detecta un poco raro
                    self.actual_people[classified] = time.time()
                else:  # Reconoce perfectamente
                    if (
                        classified not in self.actual_people
                        or (time.time() - self.actual_people[classified]) > 30
                    ):
                        self.read_text("Bienvenido de vuelta " + classified)

                    self.actual_people[classified] = time.time()
                    output = self.training_request(
                        String(data="refine_class"),
                        String(data=classified),
                        features_msg,
                        pos_msg,
                    )
                    if output < 0:
                        self.node.get_logger().info(">> ERROR: Al refinar una clase")      
            
            #Si el clasificado es dummy y lo ve bien
            elif classified == "dummy" and distance > self.UPPER_BOUND:
                if self.asking_name:
                    self.node.publisher_face_name.publish(
                        FaceGuiRequest(
                            mode=0,
                            face=face_aligned_msg
                        )
                    )
                else:
                    self.tracking = False
                    self.asking_name = False
                    self.node.get_logger().info(">> Te voy a guardar")
                    self.training_request(
                        String(data="delete_class"), String(data="dummy"), [], 0
                    )
                    # res = self.tracking_request()
                    # print(str(res.data))

                    classified = self.face_name
                    self.face_name = None
                    if classified is not None:
                        self.actual_people[classified] = time.time()
                        already_known = self.training_request(
                            String(data="add_class"),
                            String(data=classified),
                            features_msg,
                            pos_msg,
                        )
                        distance = 1

                        if already_known == 1:
                            self.read_text(
                                "Perdona "
                                + classified
                                + ", no te había reconocido bien"
                            )
                        elif already_known == 0:
                            self.read_text(
                                "Bienvenido " + classified + ", no te conocía"
                            )
                        else:
                            self.node.get_logger().info(
                                ">> ERROR: Algo salio mal al agregar una nueva clase"
                            )

                self.last_tracking_face_timer = time.time()
                self.track_face(positions[i])
                output = self.training_request(
                    String(data="refine_class"),
                    String(data=classified),
                    features_msg,
                    pos_msg,
                )
                if output < 0:
                    self.node.get_logger().info(">> ERROR: Al refinar una clase")
            
            #Si esta viendo a la persona que esta trackeando
            elif classified == self.tracking_face and distance > self.UPPER_BOUND:
                if self.asking_confirmation and not self.asking_name:
                    self.node.publisher_face_name.publish(
                        FaceGuiRequest(
                            mode=1,
                            face=face_aligned_msg,
                            texto=String(data=self.tracking_face),
                        )
                    )
            
                    
                else:
                    if(self.yes_no_result == "Si"):
                        
                        self.node.get_logger().info(">> Te voy a refinar")
                        output = self.training_request(
                            String(data="add_features"),
                            String(data=self.tracking_face),
                            features_msg,
                            pos_msg,
                        )
                        self.tracking_face = ""
                        self.tracking = False
                        self.asking_confirmation = False
                        self.if_no = False
                    elif(self.yes_no_result == "No" and not self.if_no):
                        self.asking_name = True
                        self.if_no = True
                        self.read_text("Entonces, ¿Cual es tu nombre?")
                        
                    elif(self.if_no):
                        if self.asking_name:
                            self.node.publisher_face_name.publish(
                                FaceGuiRequest(
                                    mode=0,
                                    face=face_aligned_msg
                                )
                            ) 
                        else:                       
                            self.tracking_face = ""
                            self.tracking = False
                            self.asking_confirmation = False
                            self.if_no = False
                            self.asking_name = False
                            
                            self.node.get_logger().info(">> Te voy a guardar")
                            
                            classified = self.face_name
                            self.face_name = None
                            if classified is not None:
                                self.actual_people[classified] = time.time()
                                already_known = self.training_request(
                                    String(data="add_class"),
                                    String(data=classified),
                                    features_msg,
                                    pos_msg,
                                )
                                distance = 1

                                if already_known == 1:
                                    self.read_text(
                                        "Perdona "
                                        + classified
                                        + ", no te había reconocido bien"
                                    )
                                elif already_known == 0:
                                    self.read_text(
                                        "Bienvenido " + classified + ", no te conocía"
                                    )
                                else:
                                    self.node.get_logger().info(
                                        ">> ERROR: Algo salio mal al agregar una nueva clase"
                                    )
                                
                        
                        
                    else:
                        print("Se chingo el codigo :(")
                        

                        
                    

                self.last_tracking_face_timer = time.time()
                self.track_face(positions[i])
                # output = self.training_request(
                #     String(data="refine_class"),
                #     String(data=classified),
                #     features_msg,
                #     pos_msg,
                # )
                # if output < 0:
                #     self.node.get_logger().info(">> ERROR: Al refinar una clase")
            
            
            mark_face(
                frame,
                positions[i],
                distance,
                self.MIDDLE_BOUND,
                self.UPPER_BOUND,
                classified=classified,
                drawRectangle=self.draw_rectangle,
                score=scores[i],
                showDistance=self.show_distance,
                showScore=self.show_score,
            )

        if self.asking_name and time.time() - self.last_tracking_face_timer > 4:
            self.tracking = False
            self.asking_name = False
            self.node.get_logger().info(">> Te he perdido de vista")
            self.training_request(
                String(data="delete_class"), String(data="dummy"), [], 0
            )
            # res = self.tracking_request()
            # print(str(res.data))
        if self.asking_confirmation and time.time() - self.last_tracking_face_timer > 4:
            self.tracking = False
            self.asking_confirmation = False
            self.tracking_face =""
            self.node.get_logger().info(">> Te he perdido de vista :-(")
            
        actual_people_json = json.dumps(self.actual_people)
        self.node.publisher_people.publish(String(data=actual_people_json))
        self.node.publisher_recognition.publish(
            self.node.br.cv2_to_imgmsg(frame, "bgr8")
        )
    def nuevaLogica2(self, frame_msg):
        """Performs all the logic. Process the frame detecting and recognizing faces on the frame.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format
        """

        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")

        positions_msg, scores_msg = self.detection_request(frame_msg)  # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        
        #Itero sobre todas las caras que veo en un frame
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
                self.recognition_request(frame_msg, positions_msg[i])
            )  # Recogniion
            face_aligned, features, classified, distance, pos = (
                self.node.br.msg_to_recognizer(
                    face_aligned_msg,
                    features_msg,
                    classified_msg,
                    distance_msg,
                    pos_msg,
                )
            )
           
            #No esta en ningun modo especifico
            if not self.ask_name_mode and not self.ask_if_mame_mode:
                if (
                    distance < self.LOWER_BOUND
                ):  # No sabe quien es, pregunta por el nombre
                    print("solo tiene q decirlo una vez")
                    self.read_text(
                        "Hola, ¿Cual es tu nombre?"
                    )
                    
                    # Vamos a asignarle un id random que lo identificara hasta que indique su nombre
                    self.ask_name_mode = True
                    self.tracking_face_id = random.randint(0,99999)            
                    self.tracking_mode = True
                   
                    #Aprendemos al desconocido como este id random
                    self.actual_people[classified] = time.time()
                    already_known = self.training_request(
                        String(data="add_class"),
                        String(data= self.tracking_face_id),
                        features_msg,
                        pos_msg,
                    )
                    distance = 1
                    
                    # Aqui mandamos la primera instruccion de que sigua a nuestro desconocido
                    self.last_tracking_face_timer = time.time()
                    self.track_face(positions[i])
                elif (
                    distance < self.MIDDLE_BOUND            
                ):  # Cree que es alguien, pide confirmacion
                    
                    if (
                        scores[i] > 0 and self.ask_unknowns
                    ):  # Pero solo si la foto es buena
                        print("te veo bakanisimo")
                        self.asking_confirmation = True
                        self.tracking = True
                        self.tracking_face = classified

                        self.last_tracking_face_timer = time.time()
                        self.track_face(positions[i])
                    else:
                        print("te veo regulin")
                elif (
                    distance < self.UPPER_BOUND
                ):  # Sabe que es alguien pero lo detecta un poco raro
                    self.actual_people[classified] = time.time()
                else:  # Reconoce perfectamente
                    if (
                        classified not in self.actual_people
                        or (time.time() - self.actual_people[classified]) > 30
                    ):
                        self.read_text("Bienvenido de vuelta " + classified)

                    self.actual_people[classified] = time.time()
                    output = self.training_request(
                        String(data="refine_class"),
                        String(data=classified),
                        features_msg,
                        pos_msg,
                    )
                    if output < 0:
                        self.node.get_logger().info(">> ERROR: Al refinar una clase")      
            
            #Si el clasificado es nuestro desconocido y lo ve bien
            elif classified == self.tracking_face_id and distance > self.UPPER_BOUND:
                if self.asking_name:
                    self.node.publisher_face_name.publish(
                        FaceGuiRequest(
                            mode=0,
                            face=face_aligned_msg
                        )
                    )
                else:
                    self.tracking = False
                    self.asking_name = False
                    self.node.get_logger().info(">> Te voy a guardar")
                    self.training_request(
                        String(data="delete_class"), String(data="dummy"), [], 0
                    )
                    # res = self.tracking_request()
                    # print(str(res.data))

                    classified = self.face_name
                    self.face_name = None
                    if classified is not None:
                        self.actual_people[classified] = time.time()
                        already_known = self.training_request(
                            String(data="add_class"),
                            String(data=classified),
                            features_msg,
                            pos_msg,
                        )
                        distance = 1

                        if already_known == 1:
                            self.read_text(
                                "Perdona "
                                + classified
                                + ", no te había reconocido bien"
                            )
                        elif already_known == 0:
                            self.read_text(
                                "Bienvenido " + classified + ", no te conocía"
                            )
                        else:
                            self.node.get_logger().info(
                                ">> ERROR: Algo salio mal al agregar una nueva clase"
                            )

                self.last_tracking_face_timer = time.time()
                self.track_face(positions[i])
                output = self.training_request(
                    String(data="refine_class"),
                    String(data=classified),
                    features_msg,
                    pos_msg,
                )
                if output < 0:
                    self.node.get_logger().info(">> ERROR: Al refinar una clase")
            
            #Si esta viendo a la persona que esta trackeando
            elif classified == self.tracking_face and distance > self.UPPER_BOUND:
                if self.asking_confirmation and not self.asking_name:
                    self.node.publisher_face_name.publish(
                        FaceGuiRequest(
                            mode=1,
                            face=face_aligned_msg,
                            texto=String(data=self.tracking_face),
                        )
                    )
            
                    
                else:
                    if(self.yes_no_result == "Si"):
                        
                        self.node.get_logger().info(">> Te voy a refinar")
                        output = self.training_request(
                            String(data="add_features"),
                            String(data=self.tracking_face),
                            features_msg,
                            pos_msg,
                        )
                        self.tracking_face = ""
                        self.tracking = False
                        self.asking_confirmation = False
                        self.if_no = False
                    elif(self.yes_no_result == "No" and not self.if_no):
                        self.asking_name = True
                        self.if_no = True
                        self.read_text("Entonces, ¿Cual es tu nombre?")
                        
                    elif(self.if_no):
                        if self.asking_name:
                            self.node.publisher_face_name.publish(
                                FaceGuiRequest(
                                    mode=0,
                                    face=face_aligned_msg
                                )
                            ) 
                        else:                       
                            self.tracking_face = ""
                            self.tracking = False
                            self.asking_confirmation = False
                            self.if_no = False
                            self.asking_name = False
                            
                            self.node.get_logger().info(">> Te voy a guardar")
                            
                            classified = self.face_name
                            self.face_name = None
                            if classified is not None:
                                self.actual_people[classified] = time.time()
                                already_known = self.training_request(
                                    String(data="add_class"),
                                    String(data=classified),
                                    features_msg,
                                    pos_msg,
                                )
                                distance = 1

                                if already_known == 1:
                                    self.read_text(
                                        "Perdona "
                                        + classified
                                        + ", no te había reconocido bien"
                                    )
                                elif already_known == 0:
                                    self.read_text(
                                        "Bienvenido " + classified + ", no te conocía"
                                    )
                                else:
                                    self.node.get_logger().info(
                                        ">> ERROR: Algo salio mal al agregar una nueva clase"
                                    )
                                
                        
                        
                    else:
                        print("Se chingo el codigo :(")
                        

                        
                    

                self.last_tracking_face_timer = time.time()
                self.track_face(positions[i])
                # output = self.training_request(
                #     String(data="refine_class"),
                #     String(data=classified),
                #     features_msg,
                #     pos_msg,
                # )
                # if output < 0:
                #     self.node.get_logger().info(">> ERROR: Al refinar una clase")
            
            
            mark_face(
                frame,
                positions[i],
                distance,
                self.MIDDLE_BOUND,
                self.UPPER_BOUND,
                classified=classified,
                drawRectangle=self.draw_rectangle,
                score=scores[i],
                showDistance=self.show_distance,
                showScore=self.show_score,
            )

        if self.asking_name and time.time() - self.last_tracking_face_timer > 4:
            self.tracking = False
            self.asking_name = False
            self.node.get_logger().info(">> Te he perdido de vista")
            self.training_request(
                String(data="delete_class"), String(data="dummy"), [], 0
            )
            # res = self.tracking_request()
            # print(str(res.data))
        if self.asking_confirmation and time.time() - self.last_tracking_face_timer > 4:
            self.tracking = False
            self.asking_confirmation = False
            self.tracking_face =""
            self.node.get_logger().info(">> Te he perdido de vista :-(")
            
        actual_people_json = json.dumps(self.actual_people)
        self.node.publisher_people.publish(String(data=actual_people_json))
        self.node.publisher_recognition.publish()
        
    def track_face(self, positions):
        pos = FacePosition()
        x, y, w, h = positions
        pos.x = x
        pos.y = y
        pos.w = w
        pos.h = h

        self.node.publisher_tracking.publish(pos)

    def tracking_request(self):
        tracking_request = Tracking.Request()

        future_detection = self.node.tracking_client.call_async(tracking_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.text

        # def confirmation_request(self, face_aligned_msg):
        """Makes a detection request to the detection service.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format.

        Returns:
            positions (int[4][]): Array of 4 ints that determine the square surronding each face.
            scores (int[]): Array of scores of the detection in the same order as the positions.
        """

        confirmation_request = FaceGuiResponse.Request()
        confirmation_request.imagen = face_aligned_msg

        future_detection = self.node.confirmation_client.call_async(
            confirmation_request
        )
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.respuesta

    def detection_request(self, frame_msg):
        """Makes a detection request to the detection service.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format.

        Returns:
            positions (int[4][]): Array of 4 ints that determine the square surronding each face.
            scores (int[]): Array of scores of the detection in the same order as the positions.
        """

        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.node.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg):
        """Makes a recognition request to the recognition service.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format.
            position_msg (int[4]): 4 ints determining the square surronding the face.

        Returns:
            face_aligned (Image-ROS2): The face aligned horizontally.
            features (float[]): Features vector of the face.
            classified (String): Class of the recognized face.
            distance (float): Recognition score.
            pos (int): Position of the vector with the best distance.
        """

        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg

        future_recognition = self.node.recognition_client.call_async(
            recognition_request
        )
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
        """Makes a training request to the training service.

        Args:
            cmd_type_msg (String): Training type (str) in ROS2 format (String).
            classified_msg (String): Class of the face.
            features_msg (float[]): Feature vector of the face.
            pos (int): Position of the vector with best distance (used to train).

        Returns:
            response (Training.srv): Result. -1 means something went wrong. 0 means everything is okay
                and in case of cmd_type = add_class, also means that the class wasn't already known. 1 means
                that the class was already known, and means the same as 0 for cmd_type != add_class.
        """

        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.classified = classified_msg
        training_request.features = features_msg
        training_request.pos = pos_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result


def main(args=None):
    rclpy.init(args=args)

    hri_logic = HRILogic()
    hri_logic.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
