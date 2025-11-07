#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Project Name:
# Tactigon Soul - Shape
# 
# Release date: 30/09/2025
# Release version: 1.0
#
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/


from platform import system

platform_name = system()
__version__ = f"5.2.0.2-rc1"

import sys
import requests
import logging
import time
from os import path
from multiprocessing import Event, Process, Queue
from flask import Flask, render_template, send_from_directory, request
from gevent.pywsgi import WSGIServer

from typing import Optional

from .config import app_config
from .models import BASE_PATH, TACTIGON_SPEECH, TACTIGON_GEAR

from .modules.socketio import SocketApp
from .modules.braccio.extension import BraccioInterface
from .modules.braccio.manager import get_braccio_interface
from .modules.shapes.extension import ShapesApp
from .modules.zion.extension import ZionInterface
from .modules.zion.manager import get_zion_interface
from .modules.tskin.manager import load_tskin, start_tskin, TSKIN_EXTENSION
from .modules.ironboy.extension import IronBoyInterface
from .modules.ironboy.manager import get_ironboy_interface

class Server(Process):
    url: str
    port: int
    debug: bool

    def __init__(self, url: str = "localhost", port: int = 5000, debug: bool = False):
        Process.__init__(self)
        self.url = url
        self.port = port
        self.debug = debug
        self._ready_flag = Event()

    @property
    def address(self) -> str:
        return F"http://{self.url}:{self.port}"
    
    @property
    def ready(self) -> bool:
        return self._ready_flag.is_set()
    
    def serve(self):
        print(F"Serving application on {self.address}, port {self.port}")
        self.start()

        while not self._ready_flag.is_set():
            time.sleep(0.5)

        input("Type any key to exit...\n")
    
        self.stop()
        self.terminate()

    def run(self):
        logging.getLogger("bleak").setLevel(logging.INFO)
        app = self.create_app(self.debug)
        server = WSGIServer((self.url, self.port), app)
        self._ready_flag.set()
        server.serve_forever()

    def create_app(self, debug: bool = False):
        flask_app = Flask(__name__, template_folder="templates", static_folder="static")
        flask_app.config.from_object(app_config)

        with flask_app.app_context():

            socket_app = SocketApp()
            shapes_app = ShapesApp(path.join(BASE_PATH, "config", "shapes"))
            braccio_interface = BraccioInterface(path.join(BASE_PATH, "config", "braccio"))
            zion_interface = ZionInterface(path.join(BASE_PATH, "config", "zion"))
            ironboy_interface = IronBoyInterface(path.join(BASE_PATH, "config", "ironboy"))

            flask_app.debug = debug
            braccio_interface.init_app(flask_app)
            zion_interface.init_app(flask_app)
            shapes_app.init_app(flask_app)
            socket_app.init_app(flask_app)
            ironboy_interface.init_app(flask_app)

            shapes_app.braccio_interface = braccio_interface
            shapes_app.zion_interface = zion_interface
            shapes_app.ironboy_interface = ironboy_interface

            socket_app.shapes_app = shapes_app
            socket_app.braccio_interface = braccio_interface
            socket_app.ironboy_interface = ironboy_interface

            flask_app.extensions[TSKIN_EXTENSION] = None
            tskin = None
            if app_config.TSKIN:
                load_tskin(app_config.TSKIN, app_config.TSKIN_VOICE)
                tskin = start_tskin()

            if tskin:
                socket_app.setTSkin(tskin)

            from . import main
            from .modules.tskin.blueprint import bp as tskin_bp
            from .modules.shapes.blueprint import bp as shapes_bp
            from .modules.braccio.blueprint import bp as braccio_bp
            from .modules.zion.blueprint import bp as zion_bp
            from .modules.ironboy.blueprint import bp as ironboy_bp

            flask_app.register_blueprint(main.bp)
            flask_app.register_blueprint(tskin_bp)
            flask_app.register_blueprint(shapes_bp)
            flask_app.register_blueprint(braccio_bp)
            flask_app.register_blueprint(zion_bp)
            flask_app.register_blueprint(ironboy_bp)

            @flask_app.route('/favicon.ico')
            def favicon():
                return send_from_directory(path.join(flask_app.root_path, "static", "images"), 'favicon.ico', mimetype='image/vnd.microsoft.icon')

            @flask_app.errorhandler(Exception)
            def handle_exception(e):
                # now you're handling non-HTTP exceptions only
                print("Error:", e)
                return render_template("error.jinja", error=e, url=request.url, method=request.method, args=request.args, form=request.form), 500

            @flask_app.context_processor
            def inject_data():
                braccio_interface = get_braccio_interface()
                zion_interface = get_zion_interface()
                ironboy_interface = get_ironboy_interface()
                
                if braccio_interface:
                    braccio_config = braccio_interface.config
                    braccio_status = braccio_interface.running
                    braccio_connected = braccio_interface.connected
                    has_braccio = True
                else:
                    braccio_config = None
                    has_braccio = braccio_status = braccio_connected = False

                if ironboy_interface:
                    ironboy_config = ironboy_interface.config
                    ironboy_status = ironboy_interface.running
                    ironboy_connected = ironboy_interface.connected
                    has_ironboy = True
                else:
                    ironboy_config = None
                    has_ironboy = ironboy_status = ironboy_connected = False

                if zion_interface:
                    zion_config = zion_interface.config
                else:
                    zion_config = None

                return dict(
                    DEBUG=app_config.DEBUG,
                    BASE_PATH=BASE_PATH,
                    platform=sys.platform,
                    tskin_config=app_config.TSKIN,
                    hand=app_config.TSKIN.hand.value if app_config.TSKIN else None,
                    request_path=request.path,
                    braccio_config=braccio_config,
                    braccio_status=braccio_status,
                    braccio_connected=braccio_connected,
                    tactigon_speech=TACTIGON_SPEECH,
                    tactigon_gear=TACTIGON_GEAR,
                    has_braccio=has_braccio,
                    zion_config=zion_config,
                    ironboy_config=ironboy_config,
                    has_ironboy=has_ironboy,
                    ironboy_status=ironboy_status,
                    ironboy_connected=ironboy_connected,
                    voice_config=app_config.TSKIN_VOICE
                )

        return flask_app
    
    def stop(self):
        return requests.get(F"{self.address}/quit").status_code == 200
