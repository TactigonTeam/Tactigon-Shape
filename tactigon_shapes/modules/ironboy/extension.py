import json
import os
import time
from typing import Optional
from flask import Flask
from tactigon_ironboy import IronBoyConfig, IronBoy, IronBoyCommand

class IronBoyInterface:
    config_file_path: str
    config: Optional[IronBoyConfig]
    _thread: Optional[IronBoy] = None

    def __init__(self, config_file_path: str, app: Optional[Flask] = None):
        self.config_file_path = config_file_path
        self.load_config()
        
        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[IronBoyInterface.__name__] = self
   
    @property
    def configured(self) -> bool:
        return False if self.config is None else True
    
    @property
    def running(self) -> bool:
        return self._thread.running if self._thread else False
    
    @property
    def executing(self) -> bool:
        if not self._thread:
            return False

        return self._thread.executing
    
    @property
    def connected(self) -> bool:
        return bool(self._thread.connected) if self._thread else False
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    def load_config(self):
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)

                self.config = IronBoyConfig.FromJSON(config_data)
                
        else:
            self.config = None


    def save_config(self, config: IronBoyConfig):
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
            self._thread = IronBoy(self.config)
            self._thread.start()

            
    def stop(self):
        if self._thread:
            self._thread.stop()
            self._thread = None

    def command(self, command: IronBoyCommand, reps: int = 1) -> bool:
        if not self._thread:
            return False

        if not self._thread.send_command(command=command, iterations=reps):
            return False
        while True:
            if not self._thread.executing:
                return True
            if not self._thread._current_command:
                return False
            if self._thread._current_command.is_timeout:
                return False
            time.sleep(0.01)    
    
    def get_shape_blocks(self):
        return {
            "commands": [(c.name.replace("_", " ").title(), c.name) for c in IronBoyCommand if c != IronBoyCommand.ACK]
        }


    
            
