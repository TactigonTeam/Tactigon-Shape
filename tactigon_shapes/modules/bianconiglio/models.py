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

from enum import Enum
from dataclasses import dataclass

class XgbModelState(Enum):
    NOT_TRAINED = "NOT_TRAINED"
    TRAINING = "TRAINING"
    READY_TO_PREDICT = "READY_TO_PREDICT"
    PREDICTING = "PREDICTING"
    FALLBACK = "FALLBACK"
    ERROR = "ERROR"

class RAGAgentState(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    ANSWERING = "ANSWERING"
    INDEXING = "INDEXING"
    ERROR = "ERROR"

@dataclass
class ModelInfo:
   model_id: str
   created_on: str
   update_on: str
   description: str
   features: list[str]
   targets: list[str]
   state: XgbModelState

   @classmethod
   def FromJSON(cls, json: dict):
       return cls(
           model_id=json.get("model_id", ""),
           created_on=json.get("created_on", ""),
           update_on=json.get("update_on", ""),
           description=json.get("description", ""),
           features=json.get("description", []),
           targets=json.get("description", []),
           state=json.get("state", XgbModelState.NOT_TRAINED),
       )
   def toJSON(self) -> dict:
       return {
           "model_id": self.model_id,
           "created_on": self.created_on,
           "update_on": self.update_on,
           "description": self.description,
           "features": self.features,
           "targets": self.targets,
           "state": self.state,
       }
  


class DataFrameFileExtension(str, Enum):
    CSV = "csv"
    JSON = "json"

class RAGFileExtension(str, Enum):
    PDF = "pdf"
    JSON = "json"
    MD = "md"
    CSV = "csv"
    XLS = "xls"
                  
@dataclass
class BianconiglioConfig:
    url: str = "http://192.168.1.46:8000"
    chord_url: str = "http://llm.chords.cloud:11434" # TODO: Change this to the correct URL for the Chord service
    user: str = "default_user"
    context: str = "default_context"

    @classmethod
    def Default(cls):
        return cls(
        )

    @classmethod
    def FromJSON(cls, data: dict):
        return cls(
            url=data.get("url", "http://localhost:8000"),
            chord_url=data.get("chord_url", "http://llm.chords.cloud:11434"),
            user=data.get("user", "default_user"),
            context=data.get("context", "default_context")
        )

    def toJSON(self) -> dict:
        return {
            "url": self.url
        }

    def is_valid(self) -> bool:
        return True

# TODO: implementarle in futuro con metodi di salvataggio per cachearle?

@dataclass
class BianconiglioChatmessage:
    message: str
    chatId: str
    userId: str

@dataclass
class BianconiglioChatResponse:
    message: str

    @classmethod
    def FromJSON(cls, data: dict):
        return cls(
            message=data.get("message", "")
        )
    def toJSON(self) -> dict:
        return {
            "message": self.message
        }