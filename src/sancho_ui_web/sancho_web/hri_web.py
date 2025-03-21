import asyncio
import websockets
import threading
import cv2
import json

import rclpy
from rclpy.node import Node

from human_face_recognition_msgs.msg import BodyOrientation, BodyMovement, HeadOrientation
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from human_face_recognition_msgs.srv import Training

from .commands.default_commands import DefaultCommands
from .commands.help_command import HelpCommand
from .commands.rotbody_command import RotbodyCommand
from .commands.rothead_command import RotheadCommand
from .commands.movebody_command import MovebodyCommand
from .commands.dance_command import DanceCommand
from .commands.block_commands import BlockCommands
from .commands.kill_command import KillCommand
from .http_server import HTTPServer

class HRIWebNode(Node):

    def __init__(self):
        super().__init__("web")

        self.broadcast_data = {}

        self.frame_subscription = self.create_subscription(Image, "camera/color/recognition", self.frames, 1)
        self.people_subscription = self.create_subscription(String, "robot/info/actual_people", self.people, 1)
        self.human_voice_pan_subscription = self.create_subscription(String, "audio/human_voice_pan", self.human_voice_pan, 10)

        self.input_tts = self.create_publisher(String, "input_tts", 1)
        self.robot_orientation = self.create_publisher(BodyOrientation, "/robot/orientation", 1)
        self.robot_movement = self.create_publisher(BodyMovement, "/robot/body/movement", 1)
        self.head_movement = self.create_publisher(HeadOrientation, "/head/movement", 1)
        self.pub_block_head_orientation = self.create_publisher(Bool, "/robot/head/block_orientation", 1)
        self.pub_block_body_movement = self.create_publisher(Bool, "/robot/body/block_movement", 1)
        self.pub_block_body_orientation = self.create_publisher(Bool, "/robot/body/block_orientation", 1)

        self.training_client = self.create_client(Training, "recognition/training")
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Training service not available, waiting again...")
        
        self.br = CvBridge()
        self.get_logger().info("Nodo Web inicializado correctamente")

    def frames(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        frame_hex = cv2.imencode(".jpg", frame)[1].tobytes().hex() # Si hacer esto aqui fuese un problema
        self.broadcast_data["IMAGE"] = frame_hex # Se puede usar la herencia y la vinculacion dinamica para hacer un metodo process_data

    def people(self, msg):
        self.broadcast_data["PEOPLE"] = json.loads(msg.data)
    
    def human_voice_pan(self, msg):
        self.broadcast_data["AUDIO"] = json.loads(msg.data)

class HRIWeb:

    def __init__(self):
        self.clients = []

        self.command_info = json.load(open('install/sancho_web/share/sancho_web/commands/commands.json'))
        
        self.commands = {}
        self.message_schedule = []

        self.node = HRIWebNode()
        self.http_server = HTTPServer("0.0.0.0", 8080)

        self.on_enable()
    
    def set_executor(self, command, executor):
        if command in self.command_info.keys():
            self.commands[command] = executor
        else:
            self.node.get_logger().info("El comando " + command + " no está registrado en commands.json")

    def on_enable(self):
        default_commands = DefaultCommands(self)
        self.set_executor("delete", default_commands)
        self.set_executor("rename", default_commands)
        self.set_executor("clear", default_commands)
        self.set_executor("speak", default_commands)

        block_commands = BlockCommands(self)
        self.set_executor("block", block_commands)
        self.set_executor("unblock", block_commands)

        self.set_executor("help", HelpCommand(self))
        self.set_executor("rotbody", RotbodyCommand(self))
        self.set_executor("movebody", MovebodyCommand(self))
        self.set_executor("rothead", RotheadCommand(self))
        self.set_executor("dance", DanceCommand(self))
        self.set_executor("kill", KillCommand(self))

    def spin(self):
        initiated_clients = []

        while rclpy.ok():
            if self.clients:
                try:
                    for client in self.clients: # Inicializar clientes nuevos
                        if client not in initiated_clients:
                            self.init_client(client)
                            initiated_clients.append(client)

                    for type in self.node.broadcast_data.keys(): # Mandar todos los ultimos mensajes de broadcast
                        data = self.node.broadcast_data[type]
                        if data is not None:
                            asyncio.run(self.send_to_all(type, data))
                            self.node.broadcast_data[type] = None

                    if len(self.message_schedule) > 0: # Responder a las peticiones de los usuarios
                        [sender, message] = self.message_schedule.pop()
                        self.process_message(sender, message)

                except Exception as e:
                    print(">> ERROR: " + str(e))
                    
            rclpy.spin_once(self.node)

    def process_message(self, sender, message):
        if message.startswith("/"):
            command_message = message.rstrip()[1:]
            args = command_message.split(" ")

            command = args[0]
            args = args[1:] if len(args) > 0 else []

            if not command in self.commands.keys() or not self.commands[command].on_command(sender, command, args):
                self.send_message(sender, "&cComando desconocido. Usa /help para ver ayuda.")
        else:
            self.send_message(sender, message)

    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message

    def init_client(self, sender):
        message = { 
            "commands": self.command_info 
        }

        asyncio.run(self.send_to_client(sender, "INIT", message))

    def send_message(self, sender, message):
        response = { 
            "message": message 
        }

        asyncio.run(self.send_to_client(sender, "RESPONSE", response))

    async def send_to_all(self, type, message):
        tasks = [asyncio.create_task(self.send_to_client(client, type, message)) for client in self.clients]
        await asyncio.wait(tasks)

    async def send_to_client(self, client, type, message):
        try:
            message_dict = {
                "type": type, 
                "data": message
            }
            
            message_json = json.dumps(message_dict)
            await client.send(message_json)
        except Exception as e:
            print(">> ERROR AL ENVIAR: " + str(e))

    async def echo(self, websocket, _):
        self.clients.append(websocket)
        print(f">> ENTRADA: Nuevo cliente conectado: {websocket.remote_address[0]}:{websocket.remote_address[1]} (Conexiones activas: {len(self.clients)})")

        try:
            async for message in websocket:
                print(f">> LOG: Mensaje recibido desde el cliente: {message}")
                self.message_schedule.append([websocket, message])

        except Exception as e:
            print(">> ERROR: " + str(e))
        finally:
            self.clients.remove(websocket)

            print( f">> SALIDA: Cliente desconectado: {websocket.remote_address[0]}:{websocket.remote_address[1]} (Conexiones activas: {len(self.clients)})")
    
    async def socket_thread(self):
        async with websockets.serve(self.echo, "0.0.0.0", 8765):
            await asyncio.Future()  # run forever

    def execute_socket_thread(self):
        asyncio.run(self.socket_thread())


def main(args=None):
    rclpy.init(args=args)
    hri_web = HRIWeb()

    socket_thread = threading.Thread(target=hri_web.execute_socket_thread)
    socket_thread.start()

    web_thread = threading.Thread(target=hri_web.http_server.run)
    web_thread.start()

    hri_web.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
