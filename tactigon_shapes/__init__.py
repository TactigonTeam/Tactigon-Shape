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
__version__ = f"5.5.0.3"

import sys
import sys
import logging
from os import path
from flask import Flask, render_template, send_from_directory, request, current_app
from gevent.pywsgi import WSGIServer
from geventwebsocket.handler import WebSocketHandler

from tactigon_shapes.config import app_config
from tactigon_shapes.models import BASE_PATH, TACTIGON_GEAR

from tactigon_shapes.modules.socketio import SocketApp, get_socket_app
from tactigon_shapes.modules.tskin.extension import TSkinInterface
from tactigon_shapes.modules.tskin.manager import get_tskin_interface
from tactigon_shapes.modules.braccio.extension import BraccioInterface
from tactigon_shapes.modules.braccio.manager import get_braccio_interface
from tactigon_shapes.modules.shapes.extension import ShapesExtension
from tactigon_shapes.modules.zion.extension import ZionInterface
from tactigon_shapes.modules.zion.manager import get_zion_interface
# from tactigon_shapes.modules.tskin.manager import load_tskin, start_tskin, stop_tskin, TSKIN_EXTENSION
from tactigon_shapes.modules.ironboy.extension import IronBoyInterface
from tactigon_shapes.modules.ironboy.manager import get_ironboy_interface
from tactigon_shapes.modules.ros2.extension import Ros2Interface
from tactigon_shapes.modules.file_manager.extension import FileManager
from tactigon_shapes.modules.chords.extension import ChordLLMInterface, ChordMLInterface
from tactigon_shapes.modules.chords.manager import get_chord_llm_interface

from tactigon_shapes.utils.extensions import force_stop_apps

class TactigonShapes:
    url: str
    port: int
    debug: bool

    _app: Flask

    def __init__(self, url: str = "localhost", port: int = 5000, debug: bool = False):
        self.url = url
        self.port = port
        self.debug = debug

        self._app = self.create_app(self.debug)

    @property
    def address(self) -> str:
        return F"http://{self.url}:{self.port}"

    @staticmethod
    def load_blueprints(flask_app: Flask):
        from tactigon_shapes import main
        from tactigon_shapes.modules.tskin.blueprint import bp as tskin_bp
        from tactigon_shapes.modules.shapes.blueprint import bp as shapes_bp
        from tactigon_shapes.modules.braccio.blueprint import bp as braccio_bp
        from tactigon_shapes.modules.zion.blueprint import bp as zion_bp
        from tactigon_shapes.modules.ros2.blueprint import bp as ros2_bp
        from tactigon_shapes.modules.ironboy.blueprint import bp as ironboy_bp
        from tactigon_shapes.modules.chords.blueprints.chord_llm import bp as chords_bp
        from tactigon_shapes.modules.file_manager.blueprint import bp as file_manager_bp

        flask_app.register_blueprint(main.bp)
        flask_app.register_blueprint(tskin_bp)
        flask_app.register_blueprint(shapes_bp)
        flask_app.register_blueprint(braccio_bp)
        flask_app.register_blueprint(zion_bp)
        flask_app.register_blueprint(ros2_bp)
        flask_app.register_blueprint(ironboy_bp)
        flask_app.register_blueprint(chords_bp)
        flask_app.register_blueprint(file_manager_bp)

    @staticmethod
    def load_shared_context() -> dict:
        braccio_interface = get_braccio_interface()
        zion_interface = get_zion_interface()
        ironboy_interface = get_ironboy_interface()
        chord_llm_interface = get_chord_llm_interface()

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

        if chord_llm_interface:
            chord_llm_config = chord_llm_interface.config
        else:
            chord_llm_config = None

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
            tactigon_gear=TACTIGON_GEAR,
            has_braccio=has_braccio,
            zion_config=zion_config,
            ironboy_config=ironboy_config,
            has_ironboy=has_ironboy,
            ironboy_status=ironboy_status,
            ironboy_connected=ironboy_connected,
            chord_llm_config=chord_llm_config
        )

    
    def create_app(self, debug: bool = False):
        flask_app = Flask(__name__, template_folder="templates", static_folder="static")
        flask_app.config.from_object(app_config)

        with flask_app.app_context():
            self.load_blueprints(flask_app)

            flask_app.debug = debug

            socket_app = SocketApp(flask_app)
            tskin_interface = TSkinInterface(path.join(BASE_PATH, "config", "tskin"), flask_app)
            shapes_app = ShapesExtension(path.join(BASE_PATH, "config", "shapes"), flask_app)
            braccio_interface = BraccioInterface(path.join(BASE_PATH, "config", "braccio"), flask_app)
            zion_interface = ZionInterface(path.join(BASE_PATH, "config", "zion"), flask_app)
            ros2_interface = Ros2Interface(path.join(BASE_PATH, "config", "ros2"), flask_app)
            ironboy_interface = IronBoyInterface(path.join(BASE_PATH, "config", "ironboy"), flask_app)
            file_manager = FileManager(path.join(BASE_PATH, "config", "file_manager"), flask_app)
            chords_llm_interface = ChordLLMInterface(path.join(BASE_PATH, "config", "chords"), flask_app)
            chords_ml_interface = ChordMLInterface(path.join(BASE_PATH, "config", "chords"), flask_app)

            shapes_app.load_interfaces(flask_app)
            socket_app.load_interfaces(flask_app)
           
            shapes_app.autostart()

            @flask_app.route('/favicon.ico')
            def favicon():
                return send_from_directory(path.join(flask_app.root_path, "static", "images"), 'favicon.ico', mimetype='image/vnd.microsoft.icon')

            @flask_app.errorhandler(Exception)
            def handle_exception(e):
                # now you're handling non-HTTP exceptions only
                current_app.logger.error("Exception while loading the page %s %s. Exception %s", request.method, request.url, e)
                return render_template("error.jinja", error=e, url=request.url, method=request.method, args=request.args, form=request.form), 500

            flask_app.context_processor(self.load_shared_context)                

        return flask_app
    
    def serve(self):
        try:
            self._app.logger.info(f"Serving application on {self.address}, port {self.port}")
            self._server = WSGIServer(
                listener=(self.url, self.port), 
                application=self._app,
                handler_class=WebSocketHandler,
                log=self._app.logger,
            )
            self._server.serve_forever()
        except Exception as e:
            logging.error(e)
            self.stop()

    def stop(self):
        try:
            with self._app.app_context():
                force_stop_apps()
                tskin_interface = get_tskin_interface()
                if tskin_interface and tskin_interface.running:
                    tskin_interface.stop()

                app = get_socket_app()
                if app and app.running:
                    app.stop()

        except Exception as e:
            logging.error(e)
        
        sys.exit(0)