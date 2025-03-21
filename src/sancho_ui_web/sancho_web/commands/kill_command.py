import time

from human_face_recognition_msgs.msg import HeadOrientation
from std_msgs.msg import String, Bool

from .command_executor import CommandExecutor


class KillCommand(CommandExecutor):
    
    def __init__(self, web_node):
        super().__init__(web_node)

    def on_command(self, sender, command, args):
        self.web_node.send_message(sender, "Matando a &lSANCHO")
        
        self.web_node.node.pub_block_head_orientation.publish(Bool(data=True))
        self.web_node.node.pub_block_body_orientation.publish(Bool(data=True))
        time.sleep(0.5)
        self.web_node.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), force=Bool(data=True), pan=0.0, tilt=0.0))
        time.sleep(1)
        self.web_node.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), force=Bool(data=True), pan=-0.65, tilt=1.55))
        time.sleep(1)
        self.web_node.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), force=Bool(data=True), pan=0.0, tilt=0.0))
        
        self.web_node.node.pub_block_head_orientation.publish(Bool(data=False))

        return True