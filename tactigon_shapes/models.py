import sys
import json
from os import getcwd
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List

from tactigon_gear import  __version__ as tactigon_gear_version

if sys.platform != "darwin":
    from tactigon_speech import __version__ as tactigon_speech_version
else:
    tactigon_speech_version = None

from .modules.tskin.models import Hand, TSkinModel, TSkinConfig, VoiceConfig, ModelGesture, ModelTouch, OneFingerGesture

BASE_PATH = getcwd()

TACTIGON_GEAR = tactigon_gear_version
TACTIGON_SPEECH = tactigon_speech_version

@dataclass
class AppConfig(object):
    DEBUG: bool
    SECRET_KEY: str
    SEND_FILE_MAX_AGE_DEFAULT: int = 1
    MODELS: List[TSkinModel] = field(default_factory=list)
    TSKIN: Optional[TSkinConfig] = None
    TSKIN_VOICE: Optional[VoiceConfig] = None
    MAXIMIZED: bool = True
    FULLSCREEN: bool = False
    file_path: Optional[str] = None

    @classmethod
    def Default(cls, file_path):
        gestures = [
            ModelGesture("up", "Up"),
            ModelGesture("down", "Down"),
            ModelGesture("push", "Up"),
            ModelGesture("pull", "Up"),
            ModelGesture("twist", "Up"),
            ModelGesture("circle", "Up"),
            ModelGesture("swipe_r", "Up"),
            ModelGesture("swipe_l", "Up"),
        ]
        touchs = [
            ModelTouch(OneFingerGesture.SINGLE_TAP, "Tap"),
            ModelTouch(OneFingerGesture.TAP_AND_HOLD, "Tap and hold"),
        ]
        return cls(
            DEBUG=True,
            SECRET_KEY="change-me",
            MODELS=[
                TSkinModel("MODEL_01_L", Hand.LEFT, datetime.now(), gestures, touchs),
                TSkinModel("MODEL_01_R", Hand.RIGHT, datetime.now(), gestures, touchs),
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
            VoiceConfig.FromJSON(json["TSKIN_VOICE"]) if "TSKIN_VOICE" in json and json["TSKIN_VOICE"] is not None and sys.platform != "darwin" else None,
            json["MAXIMIZED"],
            json["FULLSCREEN"],
            file_path
            )
    
    def toJSON(self) -> object:
        return {
            "DEBUG": self.DEBUG,
            "SECRET_KEY": self.SECRET_KEY,
            "SEND_FILE_MAX_AGE_DEFAULT": self.SEND_FILE_MAX_AGE_DEFAULT,
            "MODELS": [m.toJSON() for m in self.MODELS],
            "TSKIN": self.TSKIN.toJSON() if self.TSKIN else None,
            "TSKIN_VOICE": self.TSKIN_VOICE.toJSON() if self.TSKIN_VOICE else None,
            "MAXIMIZED": self.MAXIMIZED,
            "FULLSCREEN": self.FULLSCREEN
        }
    
    def save(self):
        if self.file_path:
            with open(self.file_path, "w") as config_file:
                json.dump(self.toJSON(), config_file, indent=2)
        