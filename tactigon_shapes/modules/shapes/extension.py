import importlib.util
import json
import shutil
import sys
import time
from queue import Queue
from uuid import UUID, uuid4
from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum
from os import path, makedirs, environ, walk, chmod, remove
from typing import List, Optional, Tuple, Any
from pathlib import Path
import platform
import zipfile
from shutil import make_archive

from flask import Flask
from pynput.keyboard import Controller as KeyboardController

from ..braccio.extension import BraccioInterface, Wrist, Gripper
from ..zion.extension import ZionInterface
from ..tskin.models import ModelGesture, TSkin, OneFingerGesture, TwoFingerGesture, TSpeechObject
from ..tskin.manager import walk

from ...extensions.base import ExtensionThread, ExtensionApp

EXPORT_FOLDER_NAME = 'export'
IMPORT_FOLDER_NAME = 'import'
IMPORT_DESCRIPTION = "Please click 'edit code' button and click save to generate the python code."

class Severity(Enum):
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3

class ShapesPostAction(Enum):
    READ_TOUCH = 1

@dataclass
class Program:
    state: object
    code: Optional[str] = None

@dataclass
class ShapeConfig:
    id: UUID
    name: str
    created_on: datetime
    modified_on: datetime
    description: Optional[str] = None
    readonly: bool = False
    app_file: str = "program.py"

    @classmethod
    def FromJSON(cls, json):
        return cls(
            UUID(json["id"]),
            json["name"],
            datetime.fromisoformat(json["created_on"]),
            datetime.fromisoformat(json["modified_on"]),
            json["description"],
            json["readonly"]
        )

    def toJSON(self) -> dict:
        return dict(
            id=self.id.hex,
            name=self.name,
            created_on=self.created_on.isoformat(),
            modified_on=self.modified_on.isoformat(),
            description=self.description,
            readonly=self.readonly
        )
    
@dataclass
class DebugMessage:
    severity: Severity
    date: datetime
    message: str

    @classmethod
    def Debug(cls, message: str):
        return cls(
            Severity.DEBUG,
            datetime.now(),
            message
        )

    @classmethod
    def Info(cls, message: str):
        return cls(
            Severity.INFO,
            datetime.now(),
            message
        )
    
    @classmethod
    def Warning(cls, message: str):
        return cls(
            Severity.WARNING,
            datetime.now(),
            message
        )
    
    @classmethod
    def Error(cls, message: str):
        return cls(
            Severity.ERROR,
            datetime.now(),
            message
        )
    
    def toJSON(self) -> dict:
        return dict(
            severity=self.severity.name,
            date=self.date.isoformat(),
            message=self.message
        )

class LoggingQueue(Queue):
    def debug(self, msg):
        self.put_nowait(DebugMessage.Debug(msg))

    def info(self, msg):
        self.put_nowait(DebugMessage.Info(msg))

    def warning(self, msg):
        self.put_nowait(DebugMessage.Warning(msg))

    def error(self, msg):
        self.put_nowait(DebugMessage.Error(msg))

class ShapeThread(ExtensionThread):
    MODULE_NAME: str = "ShapeThreadModule"
    TOUCH_DEBOUCE_TIME: float = 0.05
    TOUCH_DEBOUNCE_TIMEOUT: float = 0.2

    _tskin: TSkin
    _keyboard: KeyboardController
    _logging_queue: LoggingQueue
    _braccio_interface: Optional[BraccioInterface] = None
    _zion_interface: Optional[ZionInterface] = None

    def __init__(self, base_path: str, app: ShapeConfig, keyboard: KeyboardController, braccio: Optional[BraccioInterface], zion: Optional[ZionInterface], logging_queue: LoggingQueue, tskin: TSkin):
        self._keyboard = keyboard
        self._tskin = tskin
        self._logging_queue = logging_queue
        self._braccio_interface = braccio
        self._zion_interface = zion

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

    def main(self):       
        actions: List[Tuple[ShapesPostAction, Any]] = []
        try:
            self.module.app(self._tskin, self._keyboard, self.braccio_interface, self.zion_interface, actions, self._logging_queue)
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


class ShapesApp(ExtensionApp):
    config_file_path: str
    config: List[ShapeConfig]
    shapes_file_path: str
    keyboard: KeyboardController
    current_id: Optional[UUID] = None
    logging_queue: LoggingQueue

    _braccio_interface: Optional[BraccioInterface] = None
    _zion_interface: Optional[ZionInterface] = None

    def __init__(self, config_path: str, flask_app: Optional[Flask] = None):
        self.config_file_path = path.join(config_path, "config.json")
        self.shapes_file_path = config_path
        self.keyboard = KeyboardController()
        self.logging_queue = LoggingQueue()

        if sys.platform == "darwin":
            self.hotkey_list = [("<ctrl>+", "ctrl"), ("<cmd>+", "cmd"), ("<shift>+", "shift"), ("<alt>+", "alt"), ("<cmd>+<alt>+", "cmd+alt"), ("<cmd>+<shift>+", "cmd+shift")]
        elif sys.platform != "darwin":
            self.hotkey_list = [("<ctrl>+", "ctrl"), ("<shift>+", "shift"), ("<alt>+", "alt"), ("<ctrl>+<alt>+", "ctrl+alt"), ("<ctrl>+<shift>+", "ctrl+shift")]
        else:
            self.hotkey_list = []
        
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

    def get_log(self) -> Optional[DebugMessage]:
        try:
            return self.logging_queue.get_nowait()
        except:
            return None

    def get_state(self, program_id: UUID) -> Optional[dict]:
        try:
            folder_path = path.join(self.shapes_file_path, "programs", program_id.hex)
            state_file_path = path.join(folder_path, "state.json")

            with open(state_file_path) as state_json_file:
                return json.load(state_json_file)

        except FileNotFoundError:
            print(f"Error: The file {self.config_file_path} does not exist.")

        except json.JSONDecodeError:
            print("Error: The file is not a valid JSON document.")

        return None

    def add(self, config: ShapeConfig, program: Optional[Program] = None) -> bool:
        self.save_config(config)

        if not program:
            program = self.get_shape()

        return self.__create_or_update_files(config.id, program)
    
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
        return self.__create_or_update_files(config.id, program)

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
            print(f"Error creating temp file for export: {e}")
            return False

    def get_downloads_path(self) -> str:
        system = platform.system()

        if system == "Windows":
            return path.join(environ["USERPROFILE"], "Downloads")
        elif system == "Darwin":
            return str(Path.home() / "Downloads")
        elif system == "Linux":
            return str(Path.home() / "Downloads")
        else:
            raise Exception("Unsupported operating system")
    
    def export(self, config: ShapeConfig) -> bool:
        unique_id = hex(int(time.time()))[2:]

        export_file_path = path.join(self.shapes_file_path, EXPORT_FOLDER_NAME, unique_id)

        if self.create_export_folder(config, export_file_path):

            export_folder_path = path.join(self.get_downloads_path(), unique_id)

            try:
                make_archive(export_folder_path, 'zip', root_dir=export_file_path)
                shutil.rmtree(path.join(self.shapes_file_path, EXPORT_FOLDER_NAME))
                return True
            except Exception as e:
                print(f"Error exporting: {e}")
                return False

    def unzip_file(self, zip_file_path, extract_to_path):
        """Unzips a zip file to a specified directory."""
        try:
            with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
                zip_ref.extractall(extract_to_path)
            return True, None
        except FileNotFoundError:
            return False, f"Error: Zip file not found at '{zip_file_path}'"
        except zipfile.BadZipFile:
            return False, f"Error: '{zip_file_path}' is not a valid zip file"
        except Exception as e:
            return False, f"An error occurred during unzipping: {e}"
        
    
    def import_shape(self, file: any) -> str:
        import_file_path = path.join(self.shapes_file_path, IMPORT_FOLDER_NAME)

        if not path.exists(import_file_path):
            makedirs(import_file_path)

        zip_file_path = path.join(import_file_path, file.filename)

        try:
            file.save(zip_file_path)
            extract_path = path.join(import_file_path, file.filename.replace('.zip', ''))

            success, error_message = self.unzip_file(zip_file_path, extract_path)

            if not success:
                shutil.rmtree(import_file_path)
                return error_message

            remove(zip_file_path)

            config_file_path = path.join(extract_path, "config.json")
            state_file_path = path.join(extract_path, "state.json")

            if not path.exists(config_file_path) or not path.exists(state_file_path):
                shutil.rmtree(import_file_path)
                return "Invalid shape format"

            with open(config_file_path, 'r') as cfg:
                data = json.load(cfg)
                config = ShapeConfig.FromJSON(data)

            config.id = uuid4()

            if self.find_shape_by_name(config.name):
                config.name = f"{config.name} - copy"

            config.description = IMPORT_DESCRIPTION

            self.save_config(config)

            destination = path.join(self.shapes_file_path, "programs", config.id.hex)
            if not path.exists(destination):
                makedirs(destination)

            shutil.move(state_file_path, destination)

            shutil.rmtree(import_file_path)
            return None

        except Exception as e:
            shutil.rmtree(import_file_path)
            return f"Error saving file: {e}"

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
    
    def get_speech_block_config(self, tspeechobject: TSpeechObject) -> list:
        args = []

        for s in tspeechobject.t_speech:
            walk(args, s)

        return args

    
    # def get_next_speechs_blocks(self, tree: Optional[List[HotWords]], *levels) -> dict:
    #     next_speechs = [(hotword.word, f"{i}_{hotword.word}_{hotword.boost}") for i, hotword in enumerate(get_next_remaining_branch(tree, *levels if levels else 0))]

    #     return {
    #         "next_speechs": [("---","")] + next_speechs,
    #     }
    
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
                    self.thread = ShapeThread(self.shapes_file_path, _config, self.keyboard, self.braccio_interface, self.zion_interface, self.logging_queue, tskin)
                    self.thread.start()
                except Exception as e:
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

    def __create_or_update_files(self, config_id: UUID, program: Program) -> bool:
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