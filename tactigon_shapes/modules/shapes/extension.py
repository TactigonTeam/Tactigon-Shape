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
import importlib.util
import json
import shutil
import sys
import time
import zipfile

from io import BytesIO
from uuid import UUID, uuid4
from queue import Queue
from os import path, makedirs
from typing import List, Optional, Tuple, Any
from pathlib import Path
from flask import Flask
from werkzeug.datastructures import FileStorage
from pynput.keyboard import Controller as KeyboardController

from tactigon_shapes.modules.shapes.models import ShapeConfig, DebugMessage, ShapesPostAction, Program
from tactigon_shapes.modules.braccio.extension import BraccioInterface, Wrist, Gripper
from tactigon_shapes.modules.zion.extension import ZionInterface
from tactigon_shapes.modules.tskin.models import ModelGesture, TSkin, OneFingerGesture, TwoFingerGesture
from tactigon_shapes.modules.ironboy.extension import IronBoyInterface
from tactigon_shapes.modules.ginos.extension import GinosInterface
from tactigon_shapes.modules.mqtt.extension import MQTTClient, mqtt_client
from tactigon_shapes.modules.ros2.extension import Ros2Interface
from tactigon_shapes.modules.ros2.models import Ros2Subscription, RosMessage, get_message_data
from tactigon_shapes.extensions.base import ExtensionThread, ExtensionApp

IMPORT_FOLDER_NAME = 'import'
IMPORT_DESCRIPTION = "Please click 'edit code' button and click save to generate the python code."

class LoggingQueue(Queue):
    def debug(self, msg: str):
        self.put_nowait(DebugMessage.Debug(msg))

    def info(self, msg: str):
        self.put_nowait(DebugMessage.Info(msg))

    def warning(self, msg: str):
        self.put_nowait(DebugMessage.Warning(msg))

    def error(self, msg: str):
        self.put_nowait(DebugMessage.Error(msg))

    def prompt(self, msg: Any):
        self.put_nowait(DebugMessage.Prompt(msg))

class ShapeThread(ExtensionThread):
    MODULE_NAME: str = "ShapeThreadModule"
    TOUCH_DEBOUCE_TIME: float = 0.05
    TOUCH_DEBOUNCE_TIMEOUT: float = 0.2

    _logger: logging.Logger
    _tskin: TSkin
    _keyboard: KeyboardController
    _logging_queue: LoggingQueue
    _braccio_interface: Optional[BraccioInterface] = None
    _zion_interface: Optional[ZionInterface] = None
    _ironboy_interface: Optional[IronBoyInterface] = None
    _ginos_interface: Optional[GinosInterface] = None
    _mqtt_interface: Optional[MQTTClient] = None
    _ros2_interface: Optional[Ros2Interface] = None
    _ros2_subscription: list[Ros2Subscription] = []
    
    def __init__(
            self, 
            base_path: str, 
            app: ShapeConfig, 
            tskin: TSkin,
            keyboard: KeyboardController, 
            braccio: Optional[BraccioInterface], 
            zion: Optional[ZionInterface], 
            ros2: Ros2Interface | None,
            ironboy: Optional[IronBoyInterface],
            logging_queue: LoggingQueue,
        ):
        self._keyboard = keyboard
        self._tskin = tskin
        self._logging_queue = logging_queue
        self._braccio_interface = braccio
        self._zion_interface = zion
        self._ros2_interface = ros2
        self._ironboy_interface = ironboy

        if app.ginos_config:
            self._ginos_interface = GinosInterface(app.ginos_config.url, app.ginos_config.model)

        if app.mqtt_config:
            self._mqtt_interface = MQTTClient(app.mqtt_config, self.on_message)

        if self._ros2_interface and app.ros2_config:
            self._ros2_subscription = app.ros2_config.subscriptions
            self._ros2_interface.start(app.ros2_config, self.on_ros2_message)

        self._logger = logging.getLogger(ShapeThread.__name__)

        ExtensionThread.__init__(self)

        self.load_module(path.join(base_path, "programs", app.id.hex, "program.py"))


    @property
    def braccio_interface(self) -> Optional[BraccioInterface]:
        return self._braccio_interface

    @braccio_interface.setter
    def braccio_interface(self, braccio_interface: Optional[BraccioInterface]):
        self._braccio_interface = braccio_interface

    @property
    def zion_interface(self) -> Optional[ZionInterface]:
        return self._zion_interface

    @zion_interface.setter
    def zion_interface(self, zion_interface: Optional[ZionInterface]):
        self._zion_interface = zion_interface

    @property
    def ros2_interface(self) -> Optional[Ros2Interface]:
        return self._ros2_interface

    @ros2_interface.setter
    def ros2_interface(self, ros2_interface: Optional[Ros2Interface]):
        self._ros2_interface = ros2_interface

    @property
    def ironboy_interface(self) -> Optional[IronBoyInterface]:
        return self.ironboy_interface

    @ironboy_interface.setter
    def ironboy_interface(self, ironboy_interface: Optional[IronBoyInterface]):
        self._ironboy_interface = ironboy_interface

    @staticmethod
    def debouce(tskin: Optional[TSkin]) -> bool:
        debouce_time = 0
        timeout = 0
        while tskin and timeout <= ShapeThread.TOUCH_DEBOUNCE_TIMEOUT:
            if tskin.touch is None:
                debouce_time += ShapeThread.TICK
            else:
                debouce_time = 0

            if debouce_time >= ShapeThread.TOUCH_DEBOUCE_TIME:
                return True
            
            time.sleep(ShapeThread.TICK)
            timeout += ShapeThread.TICK
            
        return False
    
    def on_ros2_message(self, message: RosMessage):
        if not self._ros2_interface or not self.module:
            return
        
        subscription = next((s for s in self._ros2_subscription if s.topic == message.topic), None)

        if not subscription:
            return  
        
        # Set the payload reference first
        setattr(self.module, subscription.payload_reference, get_message_data(message.msg))
        # Execute the function
        getattr(self.module, subscription.function)(self._logging_queue)
        # Clear the payload
        setattr(self.module, subscription.payload_reference, None)
    
    def on_message(self, client: mqtt_client.Client, userdata: dict, message: mqtt_client.MQTTMessage):
        if not self._mqtt_interface or not self.module:
            return
        
        subscription = next((s for s in self._mqtt_interface.config.subscriptions if s.topic == message.topic), None)

        if not subscription:
            return
        
        # Set the payload reference first
        setattr(self.module, subscription.payload_reference, json.loads(message.payload))
        # Execute the function
        getattr(self.module, subscription.function)(self._logging_queue)
        # Clear the payload
        setattr(self.module, subscription.payload_reference, None)

    def run(self):
        shape_setup_fn = getattr(self.module, "tactigon_shape_setup", None)

        if shape_setup_fn:
            try:
                self.module.tactigon_shape_setup(
                    self._tskin, 
                    self._keyboard, 
                    self.braccio_interface, 
                    self.zion_interface, 
                    self._ros2_interface,
                    self._ironboy_interface, 
                    self._ginos_interface,
                    self._mqtt_interface,
                    self._logging_queue
                )
            except Exception as e:
                self._logger.error(e)
                self._logging_queue.error(str(e))
        
        ExtensionThread.run(self)

    def main(self):
        try:
            self.module.tactigon_shape_function(
                self._tskin, 
                self._keyboard, 
                self.braccio_interface, 
                self.zion_interface, 
                self._ros2_interface,
                self._ironboy_interface, 
                self._ginos_interface,
                self._mqtt_interface,
                self._logging_queue
            )
        except Exception as e:
            self._logging_queue.error(str(e))

    def load_module(self, source: str):
        """
        reads file source and loads it as a module

        :param source: file to load
        :param module_name: name of module to register in sys.modules
        :return: loaded module
        """
        spec = importlib.util.spec_from_file_location(self.MODULE_NAME, source)
        self.module = importlib.util.module_from_spec(spec)  # type: ignore
        sys.modules[self.MODULE_NAME] = self.module
        spec.loader.exec_module(self.module)  # type: ignore

    def stop(self):
        shape_close_fn = getattr(self.module, "tactigon_shape_close", None)

        if shape_close_fn:
            try:
                self.module.tactigon_shape_close(
                    self._tskin, 
                    self._keyboard, 
                    self.braccio_interface, 
                    self.zion_interface, 
                    self._ros2_interface,
                    self._ironboy_interface, 
                    self._ginos_interface,
                    self._mqtt_interface,
                    self._logging_queue
                )
            except Exception as e:
                self._logger.error(e)

        if self._ros2_interface:
            self._ros2_interface.stop()

        if self._ginos_interface:
            self._ginos_interface = None
            
        if self._mqtt_interface:
            self._mqtt_interface.disconnect()
            self._mqtt_interface = None

        ExtensionThread.stop(self)


class ShapesApp(ExtensionApp):
    config_file_path: str
    config: List[ShapeConfig]
    shapes_file_path: str
    keyboard: KeyboardController
    current_id: Optional[UUID] = None
    logging_queue: LoggingQueue
    in_flight_log: Optional[DebugMessage] = None

    _logger: logging.Logger
    _ironboy_interface: Optional[IronBoyInterface] = None
    _braccio_interface: Optional[BraccioInterface] = None
    _zion_interface: Optional[ZionInterface] = None
    _ros2_interface: Ros2Interface | None = None

    def __init__(self, config_path: str, flask_app: Optional[Flask] = None):
        self.config_file_path = path.join(config_path, "config.json")
        self.shapes_file_path = config_path
        self.keyboard = KeyboardController()
        self.logging_queue = LoggingQueue()
        self._logger = logging.getLogger(ShapesApp.__name__)

        self.hotkey_list = [("<ctrl>+", "ctrl"), ("<shift>+", "shift"), ("<alt>+", "alt"), ("<ctrl>+<alt>+", "ctrl+alt"), ("<ctrl>+<shift>+", "ctrl+shift")]        
        self.function_list = [("Left arrow", "<left>"), ("Right arrow", "<right>"), ("Up arrow", "<up>"), ("Down arrow", "<down>"), ("Del", "<delete>"), ("Esc", "<esc>"), ("Enter", "<enter>")] + [(F"f{k+1}", F"<f{k+1}>") for k in range(12)]
        self.wristOptions = [[e.name.capitalize(), e.name] for e in Wrist]
        self.gripperOptions = [[e.name.capitalize(), e.name] for e in Gripper]

        with open(self.config_file_path) as cfg:
            self.config = [ShapeConfig.FromJSON(c) for c in json.load(cfg)]

        ExtensionApp.__init__(self, flask_app)

    @property
    def braccio_interface(self) -> Optional[BraccioInterface]:
        return self._braccio_interface

    @braccio_interface.setter
    def braccio_interface(self, braccio_interface: Optional[BraccioInterface]):
        self._braccio_interface = braccio_interface
    

    @property
    def zion_interface(self) -> Optional[ZionInterface]:
        return self._zion_interface

    @zion_interface.setter
    def zion_interface(self, zion_interface: Optional[ZionInterface]):
        self._zion_interface = zion_interface

    @property
    def ros2_interface(self) -> Optional[Ros2Interface]:
        return self._ros2_interface

    @ros2_interface.setter
    def ros2_interface(self, ros2_interface: Optional[Ros2Interface]):
        self._ros2_interface = ros2_interface

    @property
    def ironboy_interface(self) -> Optional[IronBoyInterface]:
        return self._ironboy_interface

    @ironboy_interface.setter
    def ironboy_interface(self, ironboy_interface: Optional[IronBoyInterface]):
        self._ironboy_interface = ironboy_interface

    def get_log(self) -> Optional[DebugMessage]:
        if self.in_flight_log:
            return self.in_flight_log
        
        try:
            self.in_flight_log = self.logging_queue.get_nowait()
            return self.in_flight_log
        except:
            return None
        
    def logging_read(self):
        self._logger.debug("Logging read acknowledged")
        self.in_flight_log = None

    def get_state(self, program_id: UUID) -> Optional[dict]:
        try:
            folder_path = path.join(self.shapes_file_path, "programs", program_id.hex)
            state_file_path = path.join(folder_path, "state.json")

            with open(state_file_path) as state_json_file:
                return json.load(state_json_file)

        except FileNotFoundError:
            self._logger.error("Error: The file %s does not exist.", self.config_file_path)

        except json.JSONDecodeError:
            self._logger.error("Error: The file is not a valid JSON document.")


        return None

    def add(self, config: ShapeConfig, program: Optional[Program] = None) -> bool:
        self.save_config(config)

        if not program:
            program = self.get_shape()

        return self._save_files(config.id, program)
    
    def save_config(self, config: Optional[ShapeConfig] = None):
        if config:
            found = False

            for cfg in self.config:
                if cfg.id == config.id:
                    cfg.name = config.name
                    cfg.description = cfg.description
                    cfg.readonly = config.readonly
                    cfg.app_file = config.app_file
                    cfg.created_on = config.created_on
                    cfg.modified_on = config.modified_on
                    found = True
                    break

            if not found:
                self.config.append(config)

        with open(self.config_file_path, "w") as config_file:
            json.dump([cfg.toJSON() for cfg in self.config], config_file, indent=2)

    def update(self, config: ShapeConfig, program: Program) -> bool:
        if config.description == IMPORT_DESCRIPTION:
            config.description = ""

        self.save_config(config)
        return self._save_files(config.id, program)

    def remove(self, program_id: UUID):
        filtered_programs = [c for c in self.config if c.id != program_id]

        self.config = filtered_programs

        with open(self.config_file_path, "w") as config_file:
            json.dump([cfg.toJSON() for cfg in self.config], config_file, indent=2)

        folder_path = path.join(self.shapes_file_path, "programs", program_id.hex)
        shutil.rmtree(folder_path)
        
    def create_export_folder(self, config: ShapeConfig, export_file_path: str) -> bool:
        try:

            config_output_path = path.join(export_file_path, "config.json")
            
            if not path.exists(export_file_path):
                makedirs(export_file_path)

            # source_python_file = path.join(self.shapes_file_path, "programs", config.id.hex, "program.py")
            source_state_file = path.join(self.shapes_file_path, "programs", config.id.hex, "state.json")

            with open(config_output_path, "w") as config_file:
                json.dump(config.toJSON(), config_file, indent=2)

            # shutil.copy(source_python_file, temp_file_path)
            shutil.copy(source_state_file, export_file_path)
            return True
        except Exception as e:
            self._logger.error("Error creating temp file for export: %s", e)
            return False

    def get_downloads_path(self) -> str:
            return str(Path.home() / "Downloads")
    
    def export(self, config: ShapeConfig) -> BytesIO:
        memory_file = BytesIO()

        with zipfile.ZipFile(memory_file, 'w', zipfile.ZIP_DEFLATED) as zf:
            with open(path.join(self.shapes_file_path, "programs", config.id.hex, "state.json")) as state:
                zf.writestr("state.json", state.read())
            
            zf.writestr("config.json", json.dumps(config.toJSON(), indent=2))

        memory_file.seek(0)
        return memory_file

    def unzip_file(self, zip_file_path, extract_to_path):
        """Unzips a zip file to a specified directory."""
        try:
            with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
                zip_ref.extractall(extract_to_path)
            return True, f"Successfully unzipped {zip_file_path} to {extract_to_path}"
        except FileNotFoundError:
            return False, f"Error: Zip file not found at '{zip_file_path}'"
        except zipfile.BadZipFile:
            return False, f"Error: '{zip_file_path}' is not a valid zip file"
        except Exception as e:
            return False, f"An error occurred during unzipping: {e}"
        
    def import_shape(self, file: FileStorage) -> tuple[ShapeConfig | None, str]:

        zip_bytes = file.read()
        zip_buffer = BytesIO(zip_bytes)
        
        try:
            with zipfile.ZipFile(zip_buffer, "r") as zf:
                file_list = zf.namelist()

                if "config.json" not in file_list or "state.json" not in file_list:
                    return None, "Invalid shape format"
                
                with zf.open("config.json") as config_file, zf.open("state.json") as state_file:
                    data = json.load(config_file)
                    config = ShapeConfig.FromJSON(data)

                    config.id = uuid4()
                    state_data = json.load(state_file)
                    program = Program(
                        state=state_data,
                        code=None,
                    )

                self.add(config, program)

                return config, f"Shape '{config.name}' imported successfully."
        except Exception as e:
            self._logger.error("Error importing shape: %s", e)
            return None, f"Error importing shape: {e}"

    def find_shape_by_id(self, config_id: UUID) -> Optional[ShapeConfig]:
        return next(filter(lambda x: x.id == config_id, self.config), None)

    def find_shape_by_name(self, name: str) -> Optional[ShapeConfig]:
        return next((c for c in self.config if c.name.strip().lower() == name.strip().lower()), None)

    def find_shape_by_name_and_not_id(self, name: str, config_id: UUID) -> Optional[ShapeConfig]:
        return next((c for c in self.config if c.id != config_id and c.name.strip().lower() == name.strip().lower()), None)

    def get_blocks_congfig(self, gestures: List[ModelGesture]) -> dict:
        taps = []

        _finger_gestures = [OneFingerGesture, TwoFingerGesture]

        for fin_gesture in _finger_gestures:
            taps.extend([[e.name.replace("_", " "), e.name] for e in fin_gesture if "TAP" in e.name])

        data = {
            "modKeys": self.hotkey_list,
            "funckeys": self.function_list,
            "gestures": [(g.label, g.gesture) for g in gestures],
            "taps": taps,           
            "wristOptions": self.wristOptions,
            "gripperOptions": self.gripperOptions,
        }

        return data
    
    def get_shape(self, uuid: Optional[UUID] = None) -> Program:
        code = None
        program_file = None

        if uuid:
            state_file = path.join(self.shapes_file_path, "programs", uuid.hex, "state.json")
            program_file = path.join(self.shapes_file_path, "programs", uuid.hex, "program.py")
        else:
            state_file = path.join(self.shapes_file_path, "init_state.json")

        with open(state_file) as state_json_file:
            state = json.load(state_json_file)

        if program_file:
            try:
                with open(program_file) as program_python_file:
                    code = program_python_file.read()
            except:
                pass

        return Program(state, code)

    def start(self, config_id: UUID, tskin: TSkin) -> Optional[Tuple[bool, str]]:
        if self.is_running:
            self.stop()

        for _config in self.config:
            if _config.id == config_id:

                current_program = self.get_shape(_config.id)

                if current_program.code is None:
                    return (False, "Code not found")

                self.current_id = _config.id
                try:
                    self.thread = ShapeThread(
                        base_path=self.shapes_file_path, 
                        app=_config, 
                        tskin=tskin, 
                        keyboard=self.keyboard,
                        braccio=self.braccio_interface, 
                        zion=self.zion_interface, 
                        ros2=self.ros2_interface,
                        ironboy=self.ironboy_interface, 
                        logging_queue=self.logging_queue
                    ) 
                    self.thread.start()
                except Exception as e:
                    self._logger.error("Cannot start Shape %s", e)
                    self.current_id = None
                    if self.thread:
                        try:
                            self.thread.stop()
                        except:
                            pass
                    self.thread = None
                    return (False, str(e))
                return (True, "")

        return None
    
    def stop(self):
        ExtensionApp.stop(self)
        while True:
            if not self.get_log():
                break

    def _save_files(self, config_id: UUID, program: Program) -> bool:
        folder_path = path.join(self.shapes_file_path, "programs", config_id.hex)
        python_file_path = path.join(folder_path, 'program.py')
        state_file_path = path.join(folder_path, 'state.json')

        if not path.exists(folder_path):
            makedirs(folder_path)

        if program.code:
            with open(python_file_path, "w", newline="", encoding="utf-8") as python_file:
                python_file.write(program.code)

        with open(state_file_path, "w") as state_json_file:
            json.dump(program.state, state_json_file, indent=2)
        
        return True