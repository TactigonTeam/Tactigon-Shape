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

from tactigon_shapes.interfaces.interface import InterfaceConfig


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

@dataclass
class TSkinConfigWrapper(InterfaceConfig):
    config: TSkinConfig | None = None
    socket: SocketConfig | None = None
    models: list[TSkinModel] = field(default_factory=list)
    scorers: list[Scorer] = field(default_factory=list)
    selected_scorer: str | None = None
    speech: TSpeechObject | None = None

    @classmethod
    def Default(cls, address: str, hand: Hand, name: str, model: TSkinModel):
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
            config=TSkinConfig(
                address=address,
                hand=hand,
                name=name,
                gesture_config=GestureConfig(
                    model_path=path.join("models", model.name, "model.pickle"),
                    encoder_path=path.join("models", model.name, "encoder.pickle"),
                    name=model.name,
                    created_at=model.date,
                ),
            ),
            socket=SocketConfig(
                host="0.0.0.0"
            ),
            models=[
                TSkinModel("MODEL_01_LEFT", Hand.LEFT, datetime.now(), gestures, touchs),
                TSkinModel("MODEL_01_RIGHT", Hand.RIGHT, datetime.now(), gestures, touchs),
            ],
            scorers=scorer,
            selected_scorer="shapes",
            speech=TSpeechObject(
                [
                    TSpeech(
                        [HotWord("pick"), HotWord("place")],
                        TSpeechObject(
                            [
                                TSpeech(
                                    [HotWord("position")],
                                    TSpeechObject(
                                        [
                                            TSpeech([HotWord("star"), HotWord("circle"), HotWord("square")])
                                        ]
                                    )       
                                )
                            ]
                        )
                    )
                ]
            )
        )

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            config=TSkinConfig.FromJSON(json["config"]) if "config" in json and json["config"] is not None else None,
            socket=SocketConfig.FromJSON(json["socket"]) if "socket" in json and json["socket"] is not None else None,
            models=[TSkinModel.FromJSON(f) for f in json.get("models", [])],
            scorers=[Scorer.FromJSON(s) for s in json.get("scorers", [])],
            selected_scorer=json.get("selected_scorer", None),
            speech=TSpeechObject.FromJSON(json["speech"]) if "speech" in json and json["speech"] is not None else None,
        )

    def toJSON(self) -> dict:
        return {
            "config": self.config.toJSON() if self.config else None,
            "socket": self.socket.toJSON() if self.socket else None,
            "models": [m.toJSON() for m in self.models],
            "scorers": [s.toJSON() for s in self.scorers],
            "selected_scorer": self.selected_scorer,
            "speech": self.speech.toJSON() if self.speech else None
        }