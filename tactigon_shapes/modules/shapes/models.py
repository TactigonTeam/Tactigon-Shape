from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum
from uuid import UUID

from typing import Optional

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


