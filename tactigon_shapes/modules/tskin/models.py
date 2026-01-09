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
from os import path
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Iterable

from tactigon_gear import TSkinSocket as TSkin, TSkinConfig, GestureConfig, SocketConfig
from tactigon_gear.models.tskin import Gesture, Hand, Angle, Touch, OneFingerGesture, TwoFingerGesture
from tactigon_gear.models.audio import TSpeechObject, TSpeech, HotWord


@dataclass
class ModelGesture:
    gesture: str
    label: str
    description: Optional[str] = None

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
    description: Optional[str] = None

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
    gestures: List[ModelGesture]
    touchs: List[ModelTouch]

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
