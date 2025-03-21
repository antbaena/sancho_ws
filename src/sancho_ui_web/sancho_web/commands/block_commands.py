from std_msgs.msg import Bool

from .command_executor import CommandExecutor


class BlockCommands(CommandExecutor):

    def __init__(self, web_node):
        super().__init__(web_node)

        self.motor_list = {
            "head_rotation": self.web_node.node.pub_block_head_orientation,
            "body_rotation": self.web_node.node.pub_block_body_orientation,
            "body_movement": self.web_node.node.pub_block_body_movement 
        }

    def on_command(self, sender, command, args):
        prefix = "des" if self.equals_ignore_case(command, "unblock") else ""
            
        if len(args) == 0:
            self.web_node.send_message(sender, f"No has especificado ningún motor para {prefix}bloquear. Recuerda que puedes indicar varios a la vez. " +
                "Los motores son &lhead_rotation&0, &lbody_rotation&0, &lbody_movement&0 o &lall&0.")
        else:
            msg = ""

            if "all" in args:
                msg = f"&lTodos&0 los motores han sido {prefix}bloqueados."
                for motor in self.motor_list.keys():
                    self.motor_list[motor].publish(Bool(data=self.equals_ignore_case(command, "block")))
            else:
                blocked_motors = [] # HACER ESTO PA Q NO REPITA EL EMNSAJE
                for motor in args:
                    if motor not in blocked_motors:
                        blocked_motors.append(motor)
                        if motor not in self.motor_list.keys():
                            msg += f"&l{motor}&0 no es un motor. "
                        else:
                            self.motor_list[motor].publish(Bool(data=self.equals_ignore_case(command, "block")))
                            msg += f"Motor &l{motor}&0 bloqueado. "
            
            self.web_node.send_message(sender, msg)
        
        return True