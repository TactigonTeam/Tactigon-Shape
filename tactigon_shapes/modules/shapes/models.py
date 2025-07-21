from uuid import UUID
from enum import Enum
from dataclasses import dataclass
from datetime import datetime

from typing import Any, Optional

from ..ginos.models import GinosConfig


class Severity(Enum):
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3

class MessageType(Enum):
    MESSAGE = "string"
    PROMPT = "prompt"

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
    ginos_config: Optional[GinosConfig] = None

    @classmethod
    def FromJSON(cls, json):
        return cls(
            id=UUID(json["id"]),
            name=json["name"],
            created_on=datetime.fromisoformat(json["created_on"]),
            modified_on=datetime.fromisoformat(json["modified_on"]),
            description=json["description"],
            readonly=json["readonly"],
            ginos_config=GinosConfig.FromJSON(json["ginos_config"]) if "ginos_config" in json and json["ginos_config"] is not None else None
        )

    def toJSON(self) -> dict:
        return dict(
            id=self.id.hex,
            name=self.name,
            created_on=self.created_on.isoformat(),
            modified_on=self.modified_on.isoformat(),
            description=self.description,
            readonly=self.readonly,
            ginos_config=self.ginos_config.toJSON() if self.ginos_config else None,
        )
    
@dataclass
class DebugMessage:
    severity: Severity
    date: datetime
    message: Any
    message_type: MessageType = MessageType.MESSAGE

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
    
    @classmethod
    def Prompt(cls, message: Any):
        return cls(
            Severity.DEBUG,
            datetime.now(),
            message,
            MessageType.PROMPT
        )
    
    def toJSON(self) -> dict:
        return dict(
            severity=self.severity.name,
            date=self.date.isoformat(),
            message=self.message if isinstance(self.message, str) else self.message.toJSON(),
            message_type=self.message_type.value
        )
