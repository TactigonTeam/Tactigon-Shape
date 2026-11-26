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


import json
from os import getcwd
from dataclasses import dataclass, field

from tactigon_gear import  __version__ as tactigon_gear_version
from .modules.tskin.models import Hand, TSkinModel, Scorer, TSkinConfig, SocketConfig, ModelGesture, ModelTouch, OneFingerGesture, TSpeechObject

BASE_PATH = getcwd()

TACTIGON_GEAR = tactigon_gear_version

@dataclass
class AppConfig(object):
    DEBUG: bool
    SECRET_KEY: str
    SEND_FILE_MAX_AGE_DEFAULT: int = 1
    file_path: str | None = None

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
        scorer = [
            Scorer("shapes", "shapes.scorer", "shapes.json")
        ]
        return cls(
            DEBUG=True,
            SECRET_KEY="change-me",
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
            file_path
            )
    
    def toJSON(self) -> object:
        return {
            "DEBUG": self.DEBUG,
            "SECRET_KEY": self.SECRET_KEY,
            "SEND_FILE_MAX_AGE_DEFAULT": self.SEND_FILE_MAX_AGE_DEFAULT,
        }
    
    def save(self):
        if self.file_path:
            with open(self.file_path, "w") as config_file:
                json.dump(self.toJSON(), config_file, indent=2)
        