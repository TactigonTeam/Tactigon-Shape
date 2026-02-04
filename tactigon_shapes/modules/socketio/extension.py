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

from tactigon_shapes.modules.ironboy.extension import IronBoyInterface
from tactigon_shapes.modules.braccio.extension import BraccioInterface
from tactigon_shapes.modules.shapes.extension import ShapesApp
from tactigon_shapes.modules.tskin.models import TSkin

class SocketApp(SocketIO):
    name: str = "socket_app"
    _TICK: float = 0.02
    socket_thread: Thread | None
    _stop_event: Event
    _shapes_app: ShapesApp | None = None
    _braccio_interface: BraccioInterface | None = None
    _ironboy_interface: IronBoyInterface | None = None
    _last_connection_status: bool | None

    def __init__(self, app: Flask | None = None, **kwargs):
        SocketIO.__init__(self, app, **kwargs)

        self.socket_thread = None
        self._stop_event = Event()
        self._tutorial_app = None
        self._last_connection_status = None
        self._logger = logging.getLogger(SocketApp.__name__)

        if app:
            self.init_app(app)

    def init_app(self, app: Flask, *args, **kwargs):
        SocketIO.init_app(self, app, *args, **kwargs)
        app.extensions[self.name] = self

    @property
    def is_running(self) -> bool:
        return not self._stop_event.is_set()

    @property
    def shapes_app(self) -> ShapesApp | None:
        """
        Get the Shapes App reference

        :return: Shapes App if present
        """

        return self._shapes_app
    
    @shapes_app.setter
    def shapes_app(self, app: ShapesApp) -> None:
        """
        Set the Shapes App reference

        :app: Shapes App
        """
        self._shapes_app = app

    @property
    def braccio_interface(self) -> BraccioInterface | None:
        """
        Get the BraccioInterface reference

        :return: BraccioInterface if present
        """

        return self._braccio_interface
    
    @braccio_interface.setter
    def braccio_interface(self, app: BraccioInterface) -> None:
        """
        Set the BraccioInterface reference

        :app: BraccioInterface
        """
        self._braccio_interface = app

    @property
    def ironboy_interface(self) -> IronBoyInterface | None:
        """
        Get the IronBoyInterface reference

        :return: IronBoyInterface if present
        """
        return self._ironboy_interface
    
    @ironboy_interface.setter
    def ironboy_interface(self, app: IronBoyInterface) -> None:
        """
        Set the IronBoyInterface reference

        :app: IronBoyInterface
        """
        self._ironboy_interface = app
    
    def setTSkin(self, tskin: TSkin) -> None:
        """
        Set the Tactigon Skin reference

        :tskin: Tactigon Skin reference
        """

        self._stop_event.clear()
        self.socket_thread = self.start_background_task(self.socket_emit_function, tskin)

    def stop(self):
        """
        Stop reading Tactigon Skin's data from the socket
        """
        self._stop_event.set()
        if self.socket_thread:
            self.socket_thread.join()
    
    def socket_emit_function(self, tskin: TSkin):
        logging.info("Starting SocketIO Tactigon Skin thread")
        while not self._stop_event.is_set():
            braccio_status = False
            braccio_connection = False

            ironboy_status = False
            ironboy_connection = False

            if self.braccio_interface:
                braccio_status = self.braccio_interface.running
                braccio_connection = self.braccio_interface.connected

            if self.ironboy_interface:
                ironboy_status = self.ironboy_interface.running
                ironboy_connection = self.ironboy_interface.connected


            payload = {
                "selector": tskin.selector.value if tskin.selector else None,
                "connected": tskin.connected,
                "battery": tskin.battery,
                "braccio_status": braccio_status,
                "braccio_connection": braccio_connection,
                "ironboy_status": ironboy_status,
                "ironboy_connection": ironboy_connection

            }

            self.emit("state", payload)

            if self._shapes_app and self._shapes_app.is_running:
                msg = self._shapes_app.get_log()
                if msg:
                    self.emit("logging", msg.toJSON(), callback=self._shapes_app.logging_read)
                
            self.sleep(SocketApp._TICK)  # type: ignore

        logging.info("Stopped SocketIO Tactigon Skin thread")