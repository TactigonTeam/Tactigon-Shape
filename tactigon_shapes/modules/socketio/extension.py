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

import logging

from threading import Thread, Event
from flask import Flask
from flask_socketio import SocketIO

from tactigon_shapes.interfaces.interface import get_interface
from tactigon_shapes.modules.tskin.extension import TSkinInterface
from tactigon_shapes.modules.ironboy.extension import IronBoyInterface
from tactigon_shapes.modules.braccio.extension import BraccioInterface
from tactigon_shapes.modules.shapes.extension import ShapesExtension
from tactigon_shapes.modules.tskin.models import TSkin

class SocketApp(SocketIO):
    _TICK: float = 0.02
    socket_thread: Thread | None
    _stop_event: Event
    _shapes_app: ShapesExtension | None = None
    _tskin: TSkin | None = None
    _braccio_interface: BraccioInterface | None = None
    _ironboy_interface: IronBoyInterface | None = None
    _tskin_interface: TSkinInterface | None = None
    _last_connection_status: bool | None

    def __init__(self, app: Flask | None = None, **kwargs):
        SocketIO.__init__(self, app, **kwargs)

        self.socket_thread = None
        self._stop_event = Event()
        self._last_connection_status = None
        self._logger = logging.getLogger(SocketApp.__name__)

        if app:
            self.init_app(app)

    def init_app(self, app: Flask, *args, **kwargs):
        SocketIO.init_app(self, app, *args, **kwargs)
        app.extensions[SocketApp.__name__] = self

        self._stop_event.clear()
        self.socket_thread = self.start_background_task(target=self.socket_emit_function)

    @property
    def running(self) -> bool:
        return not self._stop_event.is_set()

    def load_interfaces(self, app: Flask):
        self._braccio_interface = get_interface(BraccioInterface, app)
        self._ironboy_interface = get_interface(IronBoyInterface, app)
        self._tskin_interface = get_interface(TSkinInterface, app)

    def stop(self):
        """
        Stop reading Tactigon Skin's data from the socket
        """
        self._stop_event.set()
        if self.socket_thread:
            self.socket_thread.join()
            self.socket_thread = None

    def socket_emit_function(self):
        logging.info("Starting SocketIO Tactigon Skin thread")
        while not self._stop_event.is_set():
            braccio_status = False
            braccio_connection = False

            ironboy_status = False
            ironboy_connection = False

            if self._braccio_interface:
                braccio_status = self._braccio_interface.running
                braccio_connection = self._braccio_interface.connected

            if self._ironboy_interface:
                ironboy_status = self._ironboy_interface.running
                ironboy_connection = self._ironboy_interface.connected

            if self._tskin_interface:
                tskin_data = self._tskin_interface.get_socket_data()
            else:
                tskin_data = {}

            payload = {
                "braccio_status": braccio_status,
                "braccio_connection": braccio_connection,
                "ironboy_status": ironboy_status,
                "ironboy_connection": ironboy_connection,
                **tskin_data
            }

            self.emit("state", payload)

            if self._shapes_app:
                msg = self._shapes_app.get_log()
                if msg:
                    self.emit("logging", msg.toJSON(), callback=self._shapes_app.logging_read)

            self.sleep(SocketApp._TICK)  # type: ignore

        logging.info("Stopped SocketIO thread")