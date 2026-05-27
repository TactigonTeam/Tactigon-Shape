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
    BOOT = "BOOT"
    CHECK_MODEL = "CHECK_MODEL"
    NOT_TRAINED = "NOT_TRAINED"
    TRAINING = "TRAINING"
    READY_TO_PREDICT = "READY_TO_PREDICT"
    PREDICTING = "PREDICTING"
    FALLBACK = "FALLBACK"
    ERROR = "ERROR"

@dataclass
class BianconiglioConfig:
    url: str = "http://192.168.1.46:8000"
    train_endpoint: str = "/train_dataset"
    predict_endpoint: str = "/predict"
    status_endpoint: str = "/status"

    @classmethod
    def Default(cls):
        return cls()

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            url=json.get("url", "http://192.168.1.46:8000"),
            train_endpoint=json.get("train_endpoint", "/train_dataset"),
            predict_endpoint=json.get("predict_endpoint", "/predict"),
            status_endpoint=json.get("status_endpoint", "/status")
        )

    def toJSON(self) -> dict:
        return {
            "url": self.url,
            "train_endpoint": self.train_endpoint,
            "predict_endpoint": self.predict_endpoint,
            "status_endpoint": self.status_endpoint
        }

    def is_valid(self) -> bool:
        return True
