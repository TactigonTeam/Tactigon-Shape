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
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/


from enum import Enum
from dataclasses import dataclass

from typing import Optional

class Wrist(Enum):
    HORIZONTAL = 90
    VERTICAL = 0

class Gripper(Enum):
    OPEN = 0
    CLOSE = 73

class Command(Enum):
    MOVE = "P"
    HOME = "H"
    TURN_ON = "1"
    TURNO_OFF = "0"

class CommandStatus(Enum):
    EXECUTING = "100"
    OK = "0"
    ERROR_1 = "1"
    ERROR_2 = "2"
    OUT_OF_RANGE = "3"
    TIMEOUT = "99"

@dataclass
class BraccioConfig:
    name: str
    address: str

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
                json["name"],
                json["address"],
            )
        
    def toJSON(self) -> object:
        return {
            "name": self.name,
            "address": self.address
        }

@dataclass
class BraccioPosition:
    base: float
    shoulder: float
    elbow: float
    wrist: float
    wrist_rotation: Wrist
    gripper: Gripper

    def toString(self):
        return f"{self.base},{self.shoulder},{self.elbow},{self.wrist},{self.wrist_rotation.value},{self.gripper.value}"

@dataclass
class BraccioCommand:
    command: Command
    position: Optional[BraccioPosition] = None

    @classmethod
    def Move(cls, position):
        return cls(Command.MOVE, position)
    
    @classmethod
    def Home(cls):
        return cls(Command.HOME)
    
    @classmethod
    def On(cls):
        return cls(Command.TURN_ON)
    
    @classmethod
    def Off(cls):
        return cls(Command.TURNO_OFF)

    def toString(self):
        pos_string = "," + self.position.toString() if self.position else ""
        return f"{self.command.value}{pos_string}"