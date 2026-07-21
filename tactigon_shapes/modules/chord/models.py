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


class ChordState(Enum):
    NOT_TRAINED = "NOT_TRAINED"
    TRAINING = "TRAINING"
    READY_TO_PREDICT = "READY_TO_PREDICT"
    PREDICTING = "PREDICTING"
    FALLBACK = "FALLBACK"
    ERROR = "ERROR"


@dataclass
class ModelInfo:
   model_id: str
   created_on: str
   update_on: str
   description: str
   features: list[str]
   targets: list[str]
   state: ChordState

   @classmethod
   def FromJSON(cls, json: dict):
       return cls(
           model_id=json.get("model_id", ""),
           created_on=json.get("created_on", ""),
           update_on=json.get("update_on", ""),
           description=json.get("description", ""),
           features=json.get("description", []),
           targets=json.get("description", []),
           state=json.get("state", ChordState.NOT_TRAINED),
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


@dataclass
class ChordConfig:
    url: str = "http://192.168.1.46:8000"

    @classmethod
    def FromJSON(cls, data: dict):
        return cls(
            url=data.get("url", "http://localhost:8000")
        )
    
    def models_endpoint(self) -> str:
        return f"{self.url}/models"
    
    def train_endpoint(self) -> str:
        return f"{self.models_endpoint()}/train"
    
    def retrain_endpoint(self, model_id: str) -> str:
        return f"{self.models_endpoint()}/{model_id}/train"

    def predict_endpoint(self, model_id) -> str:
        return f"{self.models_endpoint()}/{model_id}/predict"
    
    def status_endpoint(self, model_id) -> str:
        return f"{self.models_endpoint()}/{model_id}/status"
    
    def log_endpoint(self, model_id) -> str:
        return f"{self.models_endpoint()}/{model_id}/logs"

    def toJSON(self) -> dict:
        return {
            "url": self.url
        }

    def is_valid(self) -> bool:
        return True