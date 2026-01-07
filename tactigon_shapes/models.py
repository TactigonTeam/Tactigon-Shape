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


import sys
import json
from os import getcwd
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List

from tactigon_gear import  __version__ as tactigon_gear_version
from .modules.tskin.models import Hand, TSkinModel, TSkinConfig, ModelGesture, ModelTouch, OneFingerGesture

BASE_PATH = getcwd()

TACTIGON_GEAR = tactigon_gear_version

@dataclass
class AppConfig(object):
    DEBUG: bool
    SECRET_KEY: str
    SEND_FILE_MAX_AGE_DEFAULT: int = 1
    MODELS: List[TSkinModel] = field(default_factory=list)
    TSKIN: Optional[TSkinConfig] = None
    file_path: Optional[str] = None

    @classmethod
    def Default(cls, file_path):
        gestures = [
            ModelGesture("up", "Up"),
            ModelGesture("down", "Down"),
            ModelGesture("push", "Push"),
            ModelGesture("pull", "Pull"),
            ModelGesture("twist", "Twist"),
            ModelGesture("circle", "Circle"),
            ModelGesture("swipe_r", "Swipe right"),
            ModelGesture("swipe_l", "Swipe left"),
        ]
        touchs = [
            ModelTouch(OneFingerGesture.SINGLE_TAP, "Tap"),
            ModelTouch(OneFingerGesture.TAP_AND_HOLD, "Tap and hold"),
        ]
        return cls(
            DEBUG=True,
            SECRET_KEY="change-me",
            MODELS=[
                TSkinModel("MODEL_01_LEFT", Hand.LEFT, datetime.now(), gestures, touchs),
                TSkinModel("MODEL_01_RIGHT", Hand.RIGHT, datetime.now(), gestures, touchs),
            ],
            file_path=file_path
        )

    @classmethod
    def FromFile(cls, file_path):
        with open(file_path, "r") as config_file:
            return cls.FromJSON(json.load(config_file), file_path)

    @classmethod
    def FromJSON(cls, json: dict, file_path: str):
        return cls(
            json["DEBUG"],
            json["SECRET_KEY"],
            json["SEND_FILE_MAX_AGE_DEFAULT"],
            [TSkinModel.FromJSON(f) for f in json["MODELS"]],
            TSkinConfig.FromJSON(json["TSKIN"]) if "TSKIN" in json and json["TSKIN"] is not None else None,
            file_path
            )
    
    def toJSON(self) -> object:
        return {
            "DEBUG": self.DEBUG,
            "SECRET_KEY": self.SECRET_KEY,
            "SEND_FILE_MAX_AGE_DEFAULT": self.SEND_FILE_MAX_AGE_DEFAULT,
            "MODELS": [m.toJSON() for m in self.MODELS],
            "TSKIN": self.TSKIN.toJSON() if self.TSKIN else None,
        }
    
    def save(self):
        if self.file_path:
            with open(self.file_path, "w") as config_file:
                json.dump(self.toJSON(), config_file, indent=2)
        