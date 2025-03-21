import os
import socket
import webbrowser

from flask import Flask, send_from_directory
from werkzeug.serving import make_server

class HTTPServer:
    def __init__(self, host="localhost", port=8080):
        self.host = host
        self.port = port
        
        self.app = Flask(__name__, static_folder=None) 
        self.app.add_url_rule('/', 'index', self.serve_html)
        self.app.add_url_rule('/<path:path>', 'static', self.serve_static)
        
        self.server = make_server(self.host, self.port, self.app)

    def serve_html(self):
        path = os.path.abspath(__file__)
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(path)))))
        webclient_dir = os.path.join(project_root, 'React/hri_web/dist')
        print(path)
        print(project_root)
        print(webclient_dir)
        return send_from_directory(webclient_dir, 'index.html')

    def serve_static(self, path):
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
        webclient_dir = os.path.join(project_root, 'React/hri_web/dist')
        return send_from_directory(webclient_dir, path)

    def get_local_ip(self):
        ip = "127.0.0.1"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        finally:
            s.close()
        return ip

    def stop(self):
        if self.server:
            self.server.shutdown()

    def run(self):
        webbrowser.open(f"http://{self.get_local_ip()}:8080")

        self.server.serve_forever()