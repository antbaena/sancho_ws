import json

from .command_executor import CommandExecutor
from std_msgs.msg import String


class DefaultCommands(CommandExecutor):
    
    def __init__(self, web_node):
        super().__init__(web_node)

    def on_command(self, sender, command, args):
        if self.equals_ignore_case(command, "delete"):
            if len(args) == 0:
                self.web_node.send_message(sender, "¡No has especificado un nombre!")
            else:
                _, message_data = self.web_node.training_request(String(data="delete_class"), String(data=json.dumps({
                    "class_name": args[0],
                })))
                self.web_node.send_message(sender, message_data.data)
            return True
        
        elif self.equals_ignore_case(command, "rename"):
            if len(args) == 0:
                self.web_node.send_message(sender, "No has especificado el nombre actual de la clase")
            elif len(args) == 1:
                self.web_node.send_message(sender, "No has especificado el nuevo nombre de la clase")
            else:
                _, message_data = self.web_node.training_request(String(data="rename_class"), String(data=json.dumps({
                    "old_name": args[0],
                    "new_name": args[1]
                })))
                self.web_node.send_message(sender, message_data.data)
            return True
        
        elif self.equals_ignore_case(command, "clear"):
            self.web_node.send_message(sender, "")
            return True
        
        elif self.equals_ignore_case(command, "speak"):
            if len(args) == 0:
                self.web_node.send_message(sender, "No has especificado un texto para leer")
            else:
                text = " ".join(args)

                self.web_node.node.input_tts.publish(String(data=text))
                self.web_node.send_message(sender, "Leyendo: " + text)
            return True

        return False