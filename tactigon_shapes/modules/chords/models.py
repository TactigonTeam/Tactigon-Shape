from datetime import datetime
from dataclasses import asdict, field
from enum import Enum

from pydantic import BaseModel

class ChordFileExtensionEnum(str, Enum):
    PDF = "pdf"
    JSON = "json"
    MD = "md"
    CSV = "csv"
    XLS = "xls"


class ChordAgentStateEnum(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    ANSWERING = "ANSWERING"
    INDEXING = "INDEXING"
    ERROR = "ERROR"


class APIResponseStatusEnum(str, Enum):
    OK = "ok"
    ERROR = "error"


class ChordsLLMConfig(BaseModel):
    url: str
    user: str = "default_user"

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            url=json["url"],
            user=json.get("user", "default_user")
        )


class Chat(BaseModel):
    chat_id: str
    user_id: str
    is_new: bool


class ChatStatus(BaseModel):
    chat_id: str
    status: ChordAgentStateEnum


class ChatMessage(BaseModel):
    chat_id: str
    user_id: str
    content: str
    role: str = "user"
    timestamp: int = field(default_factory=lambda: int(datetime.now().timestamp()*1000))
    category: str | None = None

    @classmethod
    def FromJSON(cls, data: dict):
        return cls(
            chat_id=data["chat_id"],
            user_id=data["user_id"],
            content=data.get("content") or data.get("message") or "",
            role=data.get("role", "user"),
            timestamp=data.get("timestamp") or int(datetime.now().timestamp()*1000),
            category=data.get("category"),
        )