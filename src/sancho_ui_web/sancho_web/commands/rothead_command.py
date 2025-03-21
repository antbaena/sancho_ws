from human_face_recognition_msgs.msg import HeadOrientation
from std_msgs.msg import String, Bool

from .command_executor import CommandExecutor


class RotheadCommand(CommandExecutor):
    
    def __init__(self, web_node):
        super().__init__(web_node)

    def on_command(self, sender, command, args):
        if len(args) == 0:
            self.web_node.send_message(sender, "No has especificado el pan")
        elif len(args) == 1:
            self.web_node.send_message(sender, "No has especificado el tilt")
        else:
            if not is_float(args[0]):
                self.web_node.send_message(sender, args[0] + " no es un pan valido")
            elif not is_float(args[1]):
                self.web_node.send_message(sender, args[1] + " no es un tilt valido")
            else:
                pan, tilt, type = float(args[0]), float(args[1]), "body"
                if len(args) > 2:
                    type = args[2]

                if type not in ["head", "body"]:
                    self.web_node.send_message(sender, "&l" + type + "&0 no es un tipo valido. Debe ser &l'head'&0 o &l'body'")     
                else:    
                    if type == "head": # PONER MODO RELATIVO Y ABSOLUTO Y EL FORCE Y  DEMAS COMO LOS OTROS
                        self.web_node.send_message(sender, "Rotando &lSOLO&0 la cabeza a &l(" + str(pan) + ", " + str(tilt) + ")") 
                        self.web_node.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), force=Bool(data=True), pan=pan, tilt=tilt)) # AHORA MISMO HACE LO MISMO LOS DOS MODOS, ARREGLAR CON ARGUMENTO FORCE O LO QUE SEA
                    elif type == "body":
                        self.web_node.send_message(sender, "Rotando la cabeza &lACOMPAÑADA CON EL CUERPO&0 a &l(" + str(pan) + ", " + str(tilt) + ")") 
                        self.web_node.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), force=Bool(data=True), pan=pan, tilt=tilt))

        return True

def is_float(str):
    try:
        float(str)
        return True
    except Exception:
        return False