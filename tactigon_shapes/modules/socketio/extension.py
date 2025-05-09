import sys
from threading import Thread, Event
from flask import Flask
from flask_socketio import SocketIO

from typing import Optional

from ..ironBoy.extension import IronBoyInterface

from ..braccio.extension import BraccioInterface
from ..shapes.extension import ShapesApp

from ..tskin.models import TSkin

class SocketApp(SocketIO):
    name: str = "socket_app"
    _TICK: float = 0.02
    socket_thread: Optional[Thread]
    _stop_event: Event
    _shapes_app: Optional[ShapesApp] = None
    _braccio_interface: Optional[BraccioInterface] = None
    _ironBoy_interface: Optional[IronBoyInterface] = None
    _last_connection_status: Optional[bool]

    def __init__(self, app: Optional[Flask] = None, **kwargs):
        SocketIO.__init__(self, app, **kwargs)

        self.socket_thread = None
        self._stop_event = Event()
        self._tutorial_app = None
        self._last_connection_status = None

        if app:
            self.init_app(app)

    def init_app(self, app: Flask, *args, **kwargs):
        SocketIO.init_app(self, app, *args, **kwargs)
        app.extensions[self.name] = self

    @property
    def is_running(self) -> bool:
        return not self._stop_event.is_set()

    @property
    def shapes_app(self) -> Optional[ShapesApp]:
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
    def braccio_interface(self) -> Optional[BraccioInterface]:
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
    def ironBoy_interface(self) -> Optional[IronBoyInterface]:

        return self._ironBoy_interface
    
    @ironBoy_interface.setter
    def ironBoy_interface(self, app: IronBoyInterface) -> None:

        self._ironBoy_interface = app
    
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

    def socket_emit_function(self, tskin: TSkin):
        while not self._stop_event.is_set():
            braccio_status = False
            braccio_connection = False

            ironBoy_status = False
            ironBoy_connection = False

            if self.braccio_interface:
                braccio_status = self.braccio_interface.running
                braccio_connection = self.braccio_interface.connected

            if self.ironBoy_interface:
                ironBoy_status = self.ironBoy_interface.running
                ironBoy_connection = self.ironBoy_interface.connected


            payload = {
                "selector": tskin.selector.value if tskin.selector else None,
                "connected": tskin.connected,
                "battery": tskin.battery,
                "braccio_status": braccio_status,
                "braccio_connection": braccio_connection,
                "ironBoy_status": ironBoy_status,
                "ironBoy_connection": ironBoy_connection

            }

            self.emit("state", payload)

            if self._shapes_app and self._shapes_app.is_running:
                msg = self._shapes_app.get_log()
                if msg:
                    self.emit("logging", msg.toJSON())
                
            self.sleep(SocketApp._TICK)  # type: ignore