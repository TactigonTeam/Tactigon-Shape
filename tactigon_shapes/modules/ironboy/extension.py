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


import time
from tactigon_ironboy import IronBoy, IronBoyCommand

from tactigon_shapes.modules.ironboy.models import IronBoyConfigWrapper
from tactigon_shapes.interfaces.interface import Interface

class IronBoyInterface(Interface):
    _config: IronBoyConfigWrapper | None
    _thread: IronBoy | None = None

    config_class = IronBoyConfigWrapper

    @property
    def can_run(self) -> bool:
        return self._config is not None
    
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

    def start(self):
        if self._thread:
            self.stop()

        if self._config:
            self._thread = IronBoy(self._config)
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


    
            
