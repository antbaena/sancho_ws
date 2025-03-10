from .command_executor import CommandExecutor


class HelpCommand(CommandExecutor):
    
    def __init__(self, web_node):
        super().__init__(web_node)

    def on_command(self, sender, command, args):
        self.web_node.send_message(sender, "Dirigete a la pestaña de Comandos para más información")
        return True