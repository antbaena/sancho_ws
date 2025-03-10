import time
from human_face_recognition_msgs.msg import BodyOrientation
from std_msgs.msg import String

from .command_executor import CommandExecutor


class DanceCommand(CommandExecutor):
    
    def __init__(self, web_node):
        super().__init__(web_node)

    def on_command(self, sender, command, args):
        self.web_node.send_message(sender, "Robot bailando: Este proceso durará 2 segundos")

        self.web_node.node.robot_orientation.publish(BodyOrientation(mode=String(data="relative"), rot_z=0.5))
        time.sleep(1)
        self.web_node.node.robot_orientation.publish(BodyOrientation(mode=String(data="relative"), rot_z=-0.5))
        time.sleep(1)

        self.web_node.send_message(sender, "Baile terminado")

        return True