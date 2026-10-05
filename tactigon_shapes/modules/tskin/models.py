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


import os
import sys
import json
from os import path
from datetime import datetime, time
from dataclasses import dataclass, field

from tactigon_gear import TSkinSocket as TSkin, TSkinConfig, GestureConfig, SocketConfig
from tactigon_gear.models.tskin import Gesture, Hand, Angle, Touch, OneFingerGesture, TwoFingerGesture
from tactigon_gear.models.audio import TSpeechObject, TSpeech, HotWord
from tactigon_shapes.modules.file_manager.extension import FileManager, FileItem
from tactigon_shapes.modules.file_manager.models import DirectoryItem, ItemAlreadyExists
from tactigon_shapes.modules.file_manager.models import DirectoryItem

@dataclass
class ModelGesture:
    gesture: str
    label: str
    description: str | None = None

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            json["gesture"],
            json["label"],
            json["description"] if "description" in json and json["description"] else None
        )

    def toJSON(self) -> dict:
        return {
            "gesture": self.gesture,
            "label": self.label,
        }

@dataclass
class ModelTouch:
    gesture: OneFingerGesture
    label: str
    description: str | None = None

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            OneFingerGesture(json["gesture"]),
            json["label"],
            json["description"] if "description" in json and json["description"] else None
        )
    
    def toJSON(self) -> dict:
        return {
            "gesture": self.gesture.value,
            "label": self.label
        }

@dataclass
class TSkinModel:
    name: str
    hand: Hand
    date: datetime
    gestures: list[ModelGesture]
    touchs: list[ModelTouch]

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            json["name"],
            Hand(json["hand"]),
            datetime.fromisoformat(json["date"]),
            [ModelGesture.FromJSON(g) for g in json["gestures"]],
            [ModelTouch.FromJSON(g) for g in json["touchs"]],
        )
    
    def toJSON(self):
        return {
            "name": self.name,
            "hand": self.hand.value,
            "date": self.date.isoformat(),
            "gestures": [g.toJSON() for g in self.gestures],
            "touchs": [t.toJSON() for t in self.touchs]
        }

@dataclass
class Scorer:
    name: str
    scorer_file: str
    speech_file: str

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            name=json["name"],
            scorer_file=json["scorer_file"],
            speech_file=json["speech_file"]
        )
    
    def toJSON(self) -> dict:
        return {
            "name": self.name,
            "scorer_file": self.scorer_file,
            "speech_file": self.speech_file,
        }

class AudioRecorder:
    DIRECTORY = "Audio_Recordings"
    EXTENSION = ".wav"
    SHARED_DIR = "/app/audio" # volume condiviso con il container speech
    
    def __init__(self, tskin: TSkin):
        self.tskin = tskin
        
    @property
    def full_filename(self, filename: str) -> str:
        name = os.path.basename(filename)
        return (name if not name.lower().endswith(self.EXTENSION) 
                else name + self.EXTENSION
                )

    def __init__(self, tskin: TSkin):
        self.tskin = tskin

    def check_filename(self, filename: str) -> str:
        name = os.path.basename(filename)
        if not name.lower().endswith(self.EXTENSION):
            name += self.EXTENSION
        return name

    def record(self, filename: str, duration: float) -> bool:
        if not self.tskin.can_listen:
            return False

        name = self.check_filename(filename)
        full_path = os.path.join(self.SHARED_DIR, name)

        if not self.tskin.record(name, duration):
            return False

        while self.tskin.is_recording:
            time.sleep(self.tskin.TICK)
            
        if not os.path.exists(full_path):
            return False

        try:
            FileManager.add_directory(self.DIRECTORY)
        except ItemAlreadyExists:
            pass

        with open(full_path, "rb") as f:
            FileManager.add_file(
                directory=self.DIRECTORY,
                file_path=name,
                content=FileStorage(stream=f, filename=name),
            )

        os.remove(full_path)
        return True