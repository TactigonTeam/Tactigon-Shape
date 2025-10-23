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
# Release date: 30/10/2025
# Release version: 1.0
#
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/


import time
import asyncio
import json
import os
from flask import Flask
from bleak import BleakClient
from threading import Thread, Event
from queue import Queue
from dataclasses import dataclass

from typing import Optional, Tuple

from .middleware import Solver
from .models import BraccioConfig, BraccioCommand, BraccioPosition, CommandStatus, Wrist, Gripper

class Braccio(Thread):
    _TICK: float = 0.1
    CMD: str = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
    STATUS: str = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

    config: BraccioConfig

    solver: Solver
    _stop_event: Event
    _cmd_status: Optional[CommandStatus] 
    client: Optional[BleakClient]
    _running: bool = False
    x_coord: float = 0
    y_coord: float = 0
    z_coord: float = 0
    base_angle: int = 0
    shoulder_angle: int = 0
    elbow_angle: int = 0
    wrist_angle: int = 0
    wrist_position: Wrist = Wrist.HORIZONTAL
    gripper_position: Gripper = Gripper.CLOSE

    def __init__(self, config: BraccioConfig):
        Thread.__init__(self, daemon=True)

        self.config = config
        self.command_queue = Queue()
        self.solver = Solver()
        self._stop_event = Event()
        self._cmd_status = None
        self.client = None

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exception_type, exception_value, exception_traceback):
        self.stop()

    @property
    def connected(self) -> bool:
        return self.client.is_connected if self.client else False
    
    @property
    def running(self) -> bool:
        return self._running
    
    @staticmethod
    def get_cmd_string(command: BraccioCommand):
        return f"{command.toString()}|"

    @staticmethod
    def get_cmd_bytes(command: BraccioCommand):
        return Braccio.get_cmd_string(command).encode()

    def run(self):
        self._running = True
        self.loop = asyncio.new_event_loop()
        self.loop.run_until_complete(self.loop.create_task(self.main()))
        self._running = False

    def stop(self, timeout: float = 5):
        self._stop_event.set()
        self.join(timeout)

    def update(self, char, data: bytearray):
        recv = data.decode()
        self._cmd_status = CommandStatus(recv)

    async def main(self):
        while not self._stop_event.is_set():
            self.client = BleakClient(self.config.address)
            try:
                await self.client.connect()
                await self.client.start_notify(Braccio.STATUS, self.update)
            except Exception as e:
                pass

            while self.client.is_connected:
                if self._stop_event.is_set():
                    await self.client.disconnect()
                    return

                command = self.get_command()
                if command:
                    self._cmd_status = CommandStatus.EXECUTING
                    command_bytes = self.get_cmd_bytes(command)
                    while command_bytes:
                        payload = command_bytes[:20]
                        command_bytes = command_bytes[20:]
                        await self.client.write_gatt_char(
                            self.CMD,
                            payload
                        )

                await asyncio.sleep(self._TICK)

            await asyncio.sleep(self._TICK)

        if self.client:
            await self.client.disconnect()

    def add_command(self, command: BraccioCommand):
        self.command_queue.put(command)

    def get_command(self) -> Optional[BraccioCommand]:
        try:
            return self.command_queue.get_nowait()
        except Exception as e:
            return None
        
    def send_command(self, command: BraccioCommand, timeout: float = 10) -> Tuple[bool, CommandStatus, float]:
        self.add_command(command)
        t = 0

        while t < timeout:
            cmd_status = self._cmd_status
            if cmd_status and cmd_status is not CommandStatus.EXECUTING:
                self._cmd_status = None
                return (cmd_status is CommandStatus.OK, cmd_status, t)
                
            t += self._TICK
            time.sleep(self._TICK) 
        
        self._cmd_status = None
        return (False, CommandStatus.TIMEOUT, t)

    def x(self, value: float):
        return self.move(value, self._y, self._z)
    
    def y(self, value: float):
        return self.move(self._x, value, self._z)
    
    def z(self, value: float):
        return self.move(self._x, self._y, value)

    def wrist(self, wrist: Wrist):
        self.wrist_position = wrist

        return self.send_command(
            BraccioCommand.Move(
                BraccioPosition(
                    self.base_angle, 
                    self.shoulder_angle, 
                    self.elbow_angle, 
                    self.wrist_angle, 
                    self.wrist_position, 
                    self.gripper_position)
                )
        )
    
    def gripper(self, gripper: Gripper):
        self.gripper_position = gripper

        return self.send_command(
            BraccioCommand.Move(
                BraccioPosition(
                    self.base_angle, 
                    self.shoulder_angle, 
                    self.elbow_angle, 
                    self.wrist_angle, 
                    self.wrist_position, 
                    self.gripper_position)
                )
        )

    def move(self, x: float, y: float, z: float, wrist: Wrist = Wrist.HORIZONTAL, gripper: Gripper = Gripper.CLOSE, timeout: float = 10) -> Tuple[bool, CommandStatus, float]:
        try:
            self.base_angle, self.shoulder_angle, self.elbow_angle, self.wrist_angle, self._x, self._y, self._z = self.solver.move_to_position_cart(x, y, z)
            self.wrist_position = wrist
            self.gripper_position = gripper
        except Exception as e:
            return (False, CommandStatus.OUT_OF_RANGE, 0)
        
        return self.send_command(
            BraccioCommand.Move(
                BraccioPosition(
                    self.base_angle, 
                    self.shoulder_angle, 
                    self.elbow_angle, 
                    self.wrist_angle, 
                    self.wrist_position, 
                    self.gripper_position)
                )
            , timeout
        )
        
    def home(self) -> Tuple[bool, CommandStatus, float]:
        return self.send_command(BraccioCommand.Home())

    def on(self) -> Tuple[bool, CommandStatus, float]:
        return self.send_command(BraccioCommand.On())

    def off(self) -> Tuple[bool, CommandStatus, float]:
        return self.send_command(BraccioCommand.Off())

class BraccioInterface:
    config_file_path: str
    config: Optional[BraccioConfig]
    _thread: Optional[Braccio] = None

    def __init__(self, config_file_path: str, app: Optional[Flask] = None):
        self.config_file_path = config_file_path
        self.load_config()
        
        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[BraccioInterface.__name__] = self
   
    @property
    def configured(self) -> bool:
        return False if self.config is None else True
    
    @property
    def running(self) -> bool:
        return self._thread.running if self._thread else False
    
    @property
    def connected(self) -> bool:
        return bool(self._thread.connected) if self._thread else False
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    def load_config(self):
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                self.config = BraccioConfig.FromJSON(json.load(f))
        else:
            self.config = None

    def save_config(self, config: BraccioConfig):
        if not os.path.exists(self.config_file_path):
            os.makedirs(self.config_file_path)

        with open(self.config_file, "w") as f:
            json.dump(config.toJSON(), f, indent=2)

        self.load_config()

    def reset_config(self):
        self.stop()
        
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            os.remove(self.config_file)

        self.load_config()

    def start(self):
        if self._thread:
            self.stop()

        if self.config:
            self._thread = Braccio(self.config)
            self._thread.start()
            self.move(0, 100, 100)
            
    def wrist(self, wrist: Wrist):
        if self._thread:
            return self._thread.wrist(wrist)

        return None
    
    def gripper(self, gripper: Gripper):
        if self._thread:
            return self._thread.gripper(gripper)
        
        return None
    
    def move(self, x: float, y: float, z: float, timeout: float = 10):
        if self._thread:
            return self._thread.move(x, y, z, self._thread.wrist_position, self._thread.gripper_position, timeout)
        
        return None
        
    def home(self):
        if self._thread:
            return self._thread.home()
        
        return None

    def on(self):
        if self._thread:
            return self._thread.on()
        
        return None

    def off(self):
        if self._thread:
            return self._thread.off()
        
        return None

    def stop(self):
        if self._thread:
            self._thread.stop()
            self._thread = None