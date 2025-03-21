import threading, time, cv2, tkinter as tk
import unidecode
from tkinter import messagebox
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from human_face_recognition_msgs.msg import FaceGuiRequest
from human_face_recognition_msgs.srv import FaceGuiResponse
from .hri_bridge import HRIBridge

class HRIGuiNode(Node):
    def __init__(self, gui):
        super().__init__("hri_gui")

        self.gui = gui

        self.time_between_requests = 3
        self.wait_for_answer = 3

        self.last_request_answered = 0
        self.last_request = 0
        self.last_tts = 0
        self.last_msg = None
        self.request_face = None
        self.timeout = False

        self.texto_acumulado = ""
        self.br = HRIBridge()

        self.timeout_sub = self.create_subscription(String, "gui/timeout", self.timeout_callback, 1)
        self.error_subs = self.create_subscription(String, "gui/error", self.error_callback, 1)
        self.warning_subs = self.create_subscription(String, "gui/warning", self.warning_callback, 1)
        self.info_subs = self.create_subscription(String, "gui/info", self.info_callback, 1)
        self.confirmation_subs = self.create_subscription(String, "gui/confirmation", self.confirmation_callback, 1)
        self.tts_sub = self.create_subscription(String, "audio/transcription", self.tts_callback, 1)
        self.gui_request_sub = self.create_subscription(FaceGuiRequest, "cara", self.gui_request_callback, 1)

        self.ask_if_name_response_client = self.create_client(FaceGuiResponse, "peticion_yesno")
        self.get_name_response_client = self.create_client(FaceGuiResponse, "peticion_nombre")

    def timeout_callback(self, msg):
        self.timeout = True

    def error_callback(self, msg):
        mensaje = msg.data
        self.gui.show_error_message(mensaje)

    def warning_callback(self, msg):
        mensaje = msg.data
        self.gui.show_warning_message(mensaje)

    def info_callback(self, msg):
        mensaje = msg.data
        self.gui.show_info_message(mensaje)

    def confirmation_callback(self, msg):
        mensaje = msg.data
        self.gui.show_confirmation_message(mensaje)

    def tts_callback(self, msg):
        self.last_tts = time.time()
        self.last_msg = msg.data
        self.texto_acumulado += msg.data + " "

    def gui_request_callback(self, msg):
        if time.time() - self.last_request_answered > self.time_between_requests: # No se pueden aceptar peticiones de caras distintas muy seguido
            self.last_request = time.time()
            self.request_face = msg
            print(">> REQUEST: Se ha recibido una peticion de " + ("¿Cual es tu nombre?" if msg.mode == 0 else "¿Eres tal?"))

class HRI_GUI:
    def __init__(self, gui):
        self.gui = gui
        self.node = HRIGuiNode(gui)

        self.names = [
            "Alejandro", "Antonio", "Alberto", "Adrián", "Andrés", "Álvaro", "Alfonso", "Benjamín", "Bruno",
            "Carlos", "Cristóbal", "Dani", "Daniel", "Diego", "David", "Eduardo", "Emilio", "Ernesto",
            "Esteban", "Fernando", "Felipe", "Francisco", "Gabriel", "Guillermo", "Gonzalo",
            "Héctor", "Hugo", "Ignacio", "Ismael", "Iván", "Javier", "Jesús", "Jorge", "José",
            "Juan", "Julián", "Leonardo", "Luis", "Manuel", "Marcos", "Mario", "Martín",
            "Mateo", "Miguel", "Nicolás", "Óscar", "Pablo", "Pedro", "Rafael", "Ramón",
            "Ricardo", "Roberto", "Rodrigo", "Rubén", "Raúl", "Salvador", "Samuel", "Santiago",
            "Sergio", "Tomás", "Vicente", "Víctor", "Paco"
            "Adriana", "Alejandra", "Alicia", "Ana", "Andrea", "Ángela", "Antonia", "Beatriz",
            "Berta", "Blanca", "Camila", "Carla", "Carmen", "Catalina", "Clara", "Cristina",
            "Daniela", "Diana", "Dolores", "Elena", "Elisa", "Emilia", "Estefanía", "Eva",
            "Fabiola", "Florencia", "Gabriela", "Gloria", "Helena", "Inés", "Irene", "Isabel",
            "Josefina", "Juana", "Julia", "Laura", "Leticia", "Lucía", "Luz", "Magdalena",
            "Manuela", "Margarita", "María", "Mariana", "Marta", "Mercedes", "Natalia",
            "Nuria", "Olga", "Patricia", "Paula", "Pilar", "Raquel", "Rocío", "Rosa",
            "Sara", "Silvia", "Sofía", "Teresa", "Valeria", "Verónica", "Victoria", "Yolanda",
            "Eulogio", "Ant", "Dominik", "Javi"
        ]

    def normalize(self, text):
        return unidecode.unidecode(text).lower().strip()

    def extract_name(self, text):
        normalized_names = {self.normalize(name): name for name in self.names}
        
        for word in text.split(" "):
            normalized_word = self.normalize(word)
            if normalized_word in normalized_names:
                return normalized_names[normalized_word]
        
        return None

    def spin(self):
        while rclpy.ok():
            if self.node.request_face is not None: # Si hay una cara que se esta pidiendo
                request_face = self.node.request_face
                if self.gui.get_active_frame_name() != "ask_if_name_frame" and self.gui.get_active_frame_name()  != "get_name_frame": # Se borra el registro del ultimo mensaje de voz
                    print("LOG >> Borrando mensajes para no responder erroneamente la peticion")
                    self.node.last_msg = None

                mode = request_face.mode
                face = self.node.br.imgmsg_to_cv2(request_face.face, "bgr8")
                if mode == 0: # Peticion de nombre
                    self.gui.get_name_frame.show(face)
                elif mode == 1: # Peticion de confirmacion
                    text = request_face.texto.data
                    self.gui.ask_if_name_frame.show(face, text)
                else:
                    print("Algo salio mal: ", mode)

            if self.gui.get_active_frame_name()  == "ask_if_name_frame": # Si estamos en el ask if name frame
                if self.node.last_msg is not None: # Si hay un mensaje de voz nuevo
                    msg = self.node.last_msg.upper()
                    if "SI" in msg or "SÍ" in msg:
                        self.gui.ask_if_name_frame.yes_no_var.set(value="Si")
                    elif "NO" in msg:
                        self.gui.ask_if_name_frame.yes_no_var.set(value="No")

                if len(self.gui.ask_if_name_frame.yes_no_var.get()): # Si ya tenemos respuesta
                    self.last_request_answered = time.time()
                    self.node.request_face = None
                    self.node.last_msg = None

                    gui_response = self.gui.ask_if_name_frame.yes_no_var.get()
                    result = self.ask_if_name_response_request(gui_response)
                    if result == 1:
                        print(">> RESPONSE: Se ha respondido correctamente al nodo logica")
                        self.gui.hello_frame.show()
                        self.gui.ask_if_name_frame.yes_no_var.set(value="")
                    else:
                        print("algo ha petao :((((((((((  en eres tal  -> " + str(result))

            elif self.gui.get_active_frame_name()  == "get_name_frame": # Si estamos en el get name frame
                if self.node.last_msg is not None: # Si hay un mensaje de voz nuevo
                    name = self.extract_name(self.node.last_msg)
                    if name is not None:
                        self.gui.get_name_frame.get_name_var.set(name)
                    else:
                        print("Ningun nombre detectado en la frase: " + self.node.last_msg)

                if len(self.gui.get_name_frame.get_name_var.get()): # Si ya tenemos respuesta
                    self.last_request_answered = time.time()
                    self.node.request_face = None
                    self.node.last_msg = None

                    gui_response = self.gui.get_name_frame.get_name_var.get()
                    result = self.get_name_response_request(gui_response)
                    if result == 1:
                        print(">> RESPONSE: Se ha respondido correctamente al nodo logica")
                        self.gui.hello_frame.show()
                        self.gui.get_name_frame.get_name_var.set(value="")
                        self.gui.get_name_frame.get_name_entry.delete("0", "end")
                    else:
                        print("algo ha petao :((((((((((  en cual es tu nombre  -> " + str(result))
            elif self.gui.get_active_frame_name() not in ["ask_if_name_frame", "get_name_frame"] and len(self.node.texto_acumulado) > 0:
                self.gui.listening_frame.show(self.node.texto_acumulado) # Si no esta haciedo nada que vaya escribiendo el audio

            # Si esta en una peticion y tarda mucho en responder
            if self.gui.get_active_frame_name() in ["ask_if_name_frame", "get_name_frame"] and time.time() - self.node.last_request > self.node.wait_for_answer:
                self.node.timeout = True
            
            # Si esta mostrando texto y no recibe nada
            if self.gui.get_active_frame_name() == "listening_frame" and time.time() - self.node.last_tts > 3:
                self.node.timeout = True

            # Si hay timeout
            if self.node.timeout:
                self.node.timeout = False
                self.node.request_face = None
                self.node.last_msg = None

                self.gui.hello_frame.show()

                self.node.texto_acumulado = ""
                self.gui.ask_if_name_frame.yes_no_var.set(value="")
                self.gui.get_name_frame.get_name_var.set(value="")
                self.gui.get_name_frame.get_name_entry.delete("0", "end")

            rclpy.spin_once(self.node, timeout_sec=0)

    def get_name_response_request(self, response):
        get_name_response_request = FaceGuiResponse.Request()
        get_name_response_request.texto = String(data=response)

        future_get_name_response = self.node.get_name_response_client.call_async(get_name_response_request)
        rclpy.spin_until_future_complete(self.node, future_get_name_response)
        result_get_name_response = future_get_name_response.result()

        return result_get_name_response.result

    def ask_if_name_response_request(self, response):
        ask_if_name_response_request = FaceGuiResponse.Request()
        ask_if_name_response_request.texto = String(data=response)

        future_ask_if_name_response = self.node.ask_if_name_response_client.call_async(ask_if_name_response_request)
        rclpy.spin_until_future_complete(self.node, future_ask_if_name_response)
        result_ask_if_name_response = future_ask_if_name_response.result()

        return result_ask_if_name_response.result

class BaseFrame():
    def __init__(self, gui, frame_name):
        self.gui = gui
        self.frame_name = frame_name

        self.frame = tk.Frame(self.gui.root)
        self.frame.place(relwidth=1, relheight=1)
    
    def show(self):
        active_frame = self.gui.active_frame
        if active_frame != self:
            print(">> LOG: Cambiando a", self.frame_name)

            if active_frame is not None:
                active_frame.frame.pack_forget() # Quitamos el frame que estaba antes

            self.frame.tkraise() # Mostramos el frame actual
            self.gui.active_frame = self
            
class HelloFrame(BaseFrame):
    def __init__(self, gui):
        super().__init__(gui, "hello_frame")
        
        self.label_hello = tk.Label(self.frame, text="Hola soy Sancho", font=("Helvetica", 36))
        self.label_hello.pack(expand=True, fill=tk.BOTH)

class ListeningFrame(BaseFrame):
    def __init__(self, gui):
        super().__init__(gui, "listening_frame")

        self.label_listening = tk.Label(self.frame, text="Te estoy escuchando:", font=("Helvetica", 36))
        self.label_listening.pack(expand=True, fill=tk.BOTH)

        self.label_text = tk.Label(self.frame, text="", font=("Helvetica", 36))
        self.label_text.bind("<Configure>", lambda e: self.label_text.config(wraplength=self.label_text.winfo_width()))
        self.label_text.pack(expand=True, fill=tk.BOTH)
    
    def show(self, text):
        self.label_text.config(text=text)

        super().show()

class AskIfNameFrame(BaseFrame):
    def __init__(self, gui):
        super().__init__(gui, "ask_if_name_frame")

        self.label_are_you = tk.Label(self.frame, text="¿Eres X?", font=("Helvetica", 50))
        self.label_are_you.pack(side=tk.TOP, pady=50)

        self.image = ImageTk.PhotoImage(Image.new("RGB", (1, 1)))
        self.label_image = tk.Label(self.frame, image=self.image, text="")
        self.label_image.pack(expand=True, fill=tk.BOTH)

        self.yes_no_var = tk.StringVar()

        self.button_yes = tk.Button(self.frame, text="Sí", font=("Helvetica", 36), bg="green", fg="white", relief="raised", activebackground="#006600", activeforeground="white", command=lambda: self.yes_no_var.set(value="Si"))
        self.button_yes.pack(side=tk.LEFT, padx=(100, 50), pady=50)
        self.button_yes.config(width=10, height=3)

        self.button_no = tk.Button(self.frame, text="No", font=("Helvetica", 36), bg="#ff0000", fg="white", relief="raised", activebackground="#990000", activeforeground="white", command=lambda: self.yes_no_var.set(value="No"))
        self.button_no.pack(side=tk.RIGHT, padx=(50, 100), pady=50)
        self.button_no.config(width=10, height=3)

    def show(self, image, name):
        self.label_are_you.config(text=f"¿Eres {name}?")
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        actual_height = int(self.gui.screen_height / 2)
        actual_width = int(actual_height * rgb_image.shape[1] / rgb_image.shape[0])
        rgb_image_resized = cv2.resize(rgb_image, (actual_width, actual_height))
        img = Image.fromarray(rgb_image_resized)
        imagen_tk = ImageTk.PhotoImage(img)

        self.label_image.config(image=imagen_tk)
        self.label_image.image = imagen_tk

        super().show()

class GetNameFrame(BaseFrame):
    def __init__(self, gui):
        super().__init__(gui, "get_name_frame")
        
        self.get_name_label = tk.Label(self.frame, text="¿Cuál es tu nombre?", font=("Helvetica", 50))
        self.get_name_label.pack(side=tk.TOP, pady=50)

        self.get_name_image = ImageTk.PhotoImage(Image.new("RGB", (1, 1)))
        self.get_name_image_label = tk.Label(self.frame, image=self.get_name_image, text="")
        self.get_name_image_label.pack(expand=True, fill=tk.BOTH)

        button_height = 3

        self.get_name_entry = tk.Entry(self.frame, font=("Helvetica", 36), relief="sunken", bd=2)
        self.get_name_entry.pack(side=tk.LEFT, padx=(50, 20), pady=(50, 50), ipady=button_height * 17, expand=True, fill=tk.X)
        self.get_name_var = tk.StringVar()

        def on_accept():
            name = self.get_name_entry.get()
            self.get_name_var.set(name)

        self.get_name_accept_button = tk.Button(self.frame, text="Aceptar", font=("Helvetica", 36), bg="green", fg="white", relief="raised", activebackground="#006600", activeforeground="white", command=on_accept)
        self.get_name_accept_button.pack(side=tk.RIGHT, padx=(20, 50), pady=50)
        self.get_name_accept_button.config(width=15, height=button_height)

    def show(self, image):
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        actual_height = int(self.gui.screen_height / 2)
        actual_width = int(actual_height * rgb_image.shape[1] / rgb_image.shape[0])
        rgb_image_resized = cv2.resize(rgb_image, (actual_width, actual_height))
        img = Image.fromarray(rgb_image_resized)
        imagen_tk = ImageTk.PhotoImage(img)

        self.get_name_image_label.config(image=imagen_tk)
        self.get_name_image_label.image = imagen_tk

        super().show()

class SimpleGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Sancho's GUI")

        self.active_frame = None
        self.screen_width = root.winfo_screenwidth()
        self.screen_height = root.winfo_screenheight()

        self.hello_frame = HelloFrame(self)
        self.listening_frame = ListeningFrame(self)
        self.ask_if_name_frame = AskIfNameFrame(self)
        self.get_name_frame = GetNameFrame(self)

        self.hello_frame.show()
    
    def get_active_frame_name(self):
        return None if self.active_frame is None else self.active_frame.frame_name
    

    def update_gui(self, name):
        self.label_hello.config(text=f"Hola {name}!")

    def update_tts(self, name):
        self.label_text.config(text=f"{name}")


    def show_error_message(self, message):
        messagebox.showerror("Error", message)

    def show_warning_message(self, message):
        messagebox.showwarning("Advertencia", message)

    def show_info_message(self, message):
        messagebox.showinfo("Información", message)

    def show_confirmation_message(self, message):
        result = messagebox.askyesno("Confirmación", message)
        return result

def main(args=None):
    rclpy.init(args=args)

    root = tk.Tk()

    gui = SimpleGUI(root)
    NODE = HRI_GUI(gui)

    thread_spin = threading.Thread(target=NODE.spin, args=())
    thread_spin.start()

    root.title("Sancho interface")
    root.mainloop()

    NODE.destroy_node()
    
    rclpy.shutdown()
    thread_spin.join()
