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
# Release date: 30/10/2025
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
from dataclasses import dataclass
from typing import Optional, List, Tuple, Iterable

from tactigon_gear import TSkinConfig, GestureConfig, Gesture, Hand, Angle, Touch, OneFingerGesture, TwoFingerGesture

if sys.platform != "darwin":
    from tactigon_speech import TSkin_Speech as OldTSkin, VoiceConfig as OldVoiceConfig, Transcription, TSpeech, TSpeechObject, HotWord
else:
    from tactigon_gear import TSkin as OldTSkin
    tactigon_speech_version = None

if sys.platform != "darwin":
    HotWords = Tuple[HotWord, Optional[Iterable["HotWords"]]]

    @dataclass
    class TranscriptionWithPath(Transcription):
        @classmethod
        def FromTranscription(cls, transcription: Transcription):
            return TranscriptionWithPath(
                transcription.text,
                transcription.path,
                transcription.time,
                transcription.timeout
            )

        def filter_tree(self, tree: Optional[TSpeechObject]) -> Optional[TSpeechObject]:
            def f(tree: Optional[TSpeechObject], tree_path: List[HotWord]) -> Optional[TSpeechObject]:
                if not tree:
                    return

                if not tree_path:
                    return tree

                node, *rest = tree_path

                branches = [branch.children for branch in tree.t_speech if node in branch.hotwords]

                if not branches:
                    return tree

                filtered_tree, *_ = branches

                return f(filtered_tree, rest)

            return f(tree, self.path) if self.path else tree
        
        def __str__(self):
            return F"TranscriptionWithPath {self.text}, Path: {', '.join([hw.word for hw in self.path] if self.path else [])}, Time: {self.time}, Timeout: {self.timeout}"
        
    @dataclass
    class VoiceConfig(OldVoiceConfig):
        voice_commands_notification: Optional[str] = path.join("config", "audio", "trigger.wav")
        voice_commands: Optional[TSpeechObject] = None

        stop_hotword = None # type: ignore

        @classmethod
        def FromJSON(cls, json: dict):
            _t = super().FromJSON(json)
            _t.stop_hotword = None # type: ignore
            _t.voice_commands = TSpeechObject.FromJSON(json["voice_commands"]) if "voice_commands"in json and json["voice_commands"] else None
            _t.voice_commands_notification = json["voice_commands_notification"] if "voice_commands_notification" in json else None
            return _t
        
        def toJSON(self) -> dict:
            d = super().toJSON()
            d["voice_commands"] = self.voice_commands.toJSON() if self.voice_commands else None
            d["voice_commands_notification"] = self.voice_commands_notification
            return d

    class TSkin(OldTSkin):
        def __init__(self, config: TSkinConfig, voice_config: Optional[VoiceConfig], debug: bool = False):
            if voice_config is None:
                raise ValueError("Missing the configuration for the voice")
            
            OldTSkin.__init__(self, config, voice_config, debug=debug)

        @property
        def can_listen(self):
            return True

else:
    @dataclass
    class VoiceConfig:
        @classmethod
        def FromJSON(cls, json: dict):
            return cls()
        
        def toJSON(self) -> dict:
            return {}
        
    @dataclass
    class HotWord:
        @classmethod
        def FromJSON(cls, json: dict):
            return cls()
        
        def toJSON(self) -> dict:
            return {}
        
    HotWords = Tuple[HotWord, Optional[Iterable["HotWords"]]]
        
    @dataclass
    class TSpeechObject:
        @classmethod
        def FromJSON(cls, json: dict):
            return cls()
        
        def toJSON(self) -> dict:
            return {}
        
    class TSkin(OldTSkin):
        def __init__(self, config: TSkinConfig, voice: Optional[VoiceConfig], debug: bool = False):
            OldTSkin.__init__(self, config, debug)

        @property
        def touch_preserve(self) -> Optional[Touch]:
            self._update_touch.acquire()
            touch = self._touch
            self._update_touch.release()
            return touch

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
