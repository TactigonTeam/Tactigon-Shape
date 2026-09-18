from datetime import datetime
from enum import Enum
from uuid import UUID

from pydantic import BaseModel, Field

# Chords LLM

class ChordLLMFileExtensionEnum(str, Enum):
    PDF = "pdf"
    JSON = "json"
    MD = "md"
    CSV = "csv"
    XLS = "xls"


class ChordLLMAgentStateEnum(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    ANSWERING = "ANSWERING"
    INDEXING = "INDEXING"
    ERROR = "ERROR"


class ChordLLMApiResponseStatusEnum(str, Enum):
    OK = "ok"
    ERROR = "error"


class MessageSchema(BaseModel):
    role: str
    content: str
    step: str | None
    created_on: datetime


class FileSchema(BaseModel):
    id: int
    file_name: str


class ChordLLMChat(BaseModel):
    chat_id: UUID
    username: str
    prompt: str | None
    title: str | None
    created_on: datetime
    modified_on: datetime
    messages: list[MessageSchema] = []
    files: list[FileSchema] = []


class ChordLLMChatStatus(BaseModel):
    chat_id: str
    status: ChordLLMAgentStateEnum


class ChordLLMChatStream(BaseModel):
    content: str


class ChordLLMChatMessage(BaseModel):
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


class ChordLLMPromptSchema(BaseModel):
    id: int
    name: str
    prompt: str
    created_on: datetime
    modified_on: datetime


class ChordLLMPromptList(BaseModel):
    prompts: list[ChordLLMPromptSchema] = []


class ChordLLMConfig(BaseModel):
    url: str = "https://localhost/"
    username: str = ""
    password: str = ""

    @classmethod
    def Default(cls):
        return cls()

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(**{k: json[k] for k in ("url", "username", "password") if k in json})
    
    def toJSON(self) -> dict:
        return {
            "url": self.url,
            "username": self.username,
            "password": self.password,
        }

    def is_valid(self) -> bool:
        return self.username != "" and self.password != "" and self.url != ""


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