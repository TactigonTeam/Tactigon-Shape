from functools import wraps
from flask import current_app

from tactigon_shapes.modules.socketio.extension import SocketApp

def get_socket_app() -> SocketApp:
    return current_app.extensions[SocketApp.name]