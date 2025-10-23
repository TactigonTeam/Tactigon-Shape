from flask import current_app

from typing import Optional

from tactigon_shapes.modules.socketio import SocketApp
from tactigon_shapes.modules.braccio.extension import BraccioInterface
from tactigon_shapes.modules.zion.extension import ZionInterface
from tactigon_shapes.modules.tskin.manager import TSKIN_EXTENSION
from tactigon_shapes.modules.ironboy.extension import IronBoyInterface

excluded_apps = [SocketApp.name, TSKIN_EXTENSION, "socketio", BraccioInterface.__name__, ZionInterface.__name__,IronBoyInterface.__name__]

def stop_apps(exclude: Optional[str] = None):
    l = excluded_apps
    if (exclude):
        l.append(exclude)

    for ext in current_app.extensions:
        if ext in l:
            continue
        current_app.extensions[ext].stop()