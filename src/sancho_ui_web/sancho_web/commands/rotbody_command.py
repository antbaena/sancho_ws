from human_face_recognition_msgs.msg import BodyOrientation
from std_msgs.msg import String, Bool

from .command_executor import CommandExecutor


class RotbodyCommand(CommandExecutor):
    
    def __init__(self, web_node):
        super().__init__(web_node)

    def on_command(self, sender, command, args):
        if len(args) == 0:
            self.web_node.send_message(sender, "No has especificado el angulo rot_z")
        else:
            if not is_float(args[0]):
                self.web_node.send_message(sender, "&l" + args[0] + "&0 no es un angulo valido")
            else:
                rot_z, mode = float(args[0]), "absolute"
                if len(args) > 1:
                    mode = args[1]

                if mode not in ["absolute", "relative"]:
                    self.web_node.send_message(sender, "&l" + mode + "&0 no es un modo valido. Debe ser &l'absolute'&0 o &l'relative'")     
                else:
                    self.web_node.node.robot_orientation.publish(BodyOrientation(mode=String(data=mode), force=Bool(data=True), rot_z=rot_z))
                    if mode == "absolute":
                        self.web_node.send_message(sender, "Rotando el cuerpo a &l" + str(rot_z)) 
                    elif mode == "relative":
                        self.web_node.send_message(sender, "Rotando el cuerpo &l" + str(rot_z) + "&0 radianes respecto a su posición actual") 

        return True

def is_float(str):
    try:
        float(str)
        return True
    except Exception:
        return False