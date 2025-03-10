from abc import ABC, abstractmethod


class CommandExecutor(ABC):
    
    def __init__(self, web_node):
        self.web_node = web_node

    @abstractmethod
    def on_command(self, sender, command, args):
        pass

    @staticmethod
    def equals_ignore_case(str1, str2):
        return str1.lower() == str2.lower()