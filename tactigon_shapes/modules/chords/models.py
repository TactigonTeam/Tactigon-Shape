from datetime import datetime
from enum import Enum

from pydantic import BaseModel, Field

# Chords LLM

class ChordsLLMFileExtensionEnum(str, Enum):
    PDF = "pdf"
    JSON = "json"
    MD = "md"
    CSV = "csv"
    XLS = "xls"


class ChordsLLMAgentStateEnum(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    ANSWERING = "ANSWERING"
    INDEXING = "INDEXING"
    ERROR = "ERROR"


class ChordsLLMAPIResponseStatusEnum(str, Enum):
    OK = "ok"
    ERROR = "error"


class ChordsLLMChat(BaseModel):
    chat_id: str
    user_id: str
    is_new: bool


class ChordsLLMChatStatus(BaseModel):
    chat_id: str
    status: ChordsLLMAgentStateEnum


class ChordsLLMChatMessage(BaseModel):
    chat_id: str
    user_id: str
    content: str
    role: str = "user"
    timestamp: int = Field(default_factory=lambda: int(datetime.now().timestamp()*1000))
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


class ChordsLLMConfig(BaseModel):
    url: str
    user: str = "default_user"

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            url=json["url"],
            user=json.get("user", "default_user")
        )


# Chords ML

class ChordsMLModelStateEnum(Enum):
    NOT_TRAINED = "NOT_TRAINED"
    TRAINING = "TRAINING"
    READY_TO_PREDICT = "READY_TO_PREDICT"
    PREDICTING = "PREDICTING"
    FALLBACK = "FALLBACK"
    ERROR = "ERROR"


class ChordsMLDFFileExtension(str, Enum):
    CSV = "csv"
    JSON = "json"


class ChordsMLModelInfo(BaseModel):
    model_id: str
    created_on: str
    update_on: str
    description: str
    features: list[str]
    targets: list[str]
    state: ChordsMLModelStateEnum

    @classmethod
    def FromJSON(cls, json: dict):
       return cls(
           model_id=json.get("model_id", ""),
           created_on=json.get("created_on", ""),
           update_on=json.get("update_on", ""),
           description=json.get("description", ""),
           features=json.get("features", []),
           targets=json.get("targets", []),
           state=ChordsMLModelStateEnum(json.get("state", "NOT_TRAINED")),
       )

class ChordsMLConfig(BaseModel):
    url: str = "http://localhost:8000"

    @classmethod
    def FromJSON(cls, data: dict):
        return cls(
            url=data.get("url", "http://localhost:8000"),
        )