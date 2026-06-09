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

class BianconiglioState(Enum):
    NOT_TRAINED = "NOT_TRAINED"
    TRAINING = "TRAINING"
    READY_TO_PREDICT = "READY_TO_PREDICT"
    PREDICTING = "PREDICTING"
    FALLBACK = "FALLBACK"
    ERROR = "ERROR"

class DataFrameFileExtension(str, Enum):
    CSV = "csv"
    JSON = "json"
                  
@dataclass
class BianconiglioConfig:
    base_endpoint: str 
    train_endpoint: str 
    retrain_endpoint: str 
    predict_endpoint: str 
    status_endpoint: str 
    log_endpoint: str 
    url: str
    

    @classmethod
    def Default(cls):
        return cls(
            base_endpoint = "/models",
            train_endpoint = "/train",
            retrain_endpoint = "/retrain",
            predict_endpoint = "/predict",
            status_endpoint = "/status",
            log_endpoint = "/logs"
        )

    @classmethod
    def FromJSON(cls, data: dict):
        return cls(
            url=data.get("url", "http://localhost:8000"),
            base_endpoint=data.get("base_endpoint", "/models"),
            train_endpoint = data.get("/train"),
            retrain_endpoint = data.get("train_endpoint","/retrain"),
            predict_endpoint = data.get("predict_endpoint", "/predict"),
            status_endpoint = data.get("status_endpoint","/status"),
            log_endpoint = data.get("log_endpoint","/logs")
        )

    def toJSON(self) -> dict:
        return {
            "url": self.url
        }

    def is_valid(self) -> bool:
        return True
