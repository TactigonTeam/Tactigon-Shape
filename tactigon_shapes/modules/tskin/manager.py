import sys
from os import path
from flask import current_app
from typing import Optional, Union

from .models import TSkin, TSkinConfig, GestureConfig, TSkinModel, VoiceConfig, Hand, TSpeech, TSpeechObject, HotWord

TSKIN_EXTENSION = "tskin"

def load_tskin(config: TSkinConfig, voice: Optional[VoiceConfig]):
    current_app.extensions[TSKIN_EXTENSION] = TSkin(config, voice)

def start_tskin():
    tskin = get_tskin()

    if tskin:
        tskin.start()
        return tskin

def stop_tskin():
    tskin = get_tskin()

    if tskin:
        tskin.terminate()

    reset_tskin()

def get_tskin() -> Optional[TSkin]:
    if TSKIN_EXTENSION in current_app.extensions:
        if isinstance(current_app.extensions[TSKIN_EXTENSION], TSkin):
            return current_app.extensions[TSKIN_EXTENSION]

    return None

def reset_tskin():
    current_app.extensions[TSKIN_EXTENSION] = None

def get_tskin_default_config(address: str, hand: Hand, name: str, model: TSkinModel):
    return TSkinConfig(
        address=address,
        hand=hand,
        name=name,
        gesture_config=GestureConfig(
            model_path=path.join("models", model.name, "model.pickle"),
            encoder_path=path.join("models", model.name, "encoder.pickle"),
            name=model.name,
            created_at=model.date,
            gestures=[g.gesture for g in model.gestures]
        )
    )

if sys.platform != "darwin":
    def walk(args, s: TSpeech, level: int = 0, parent: str = "_init_"):
        if level > len(args) - 1:
            args.append(dict())

        if parent not in args[level]:
            args[level][parent] = list(["---"] if level > 0 else [])

        for hw in s.hotwords:
            if hw.word not in args[level][parent]:
                args[level][parent].append(hw.word)
            if s.children:
                for child in s.children.t_speech:
                    walk(args, child, level + 1, hw.word)

    def get_voice_default_config() -> Optional[VoiceConfig]:
        return VoiceConfig(
            path.join("speech", "deepspeech-0.9.3-models.tflite"),
            path.join("speech", "shapes.scorer"),
            voice_timeout=8,
            silence_timeout=3,
            voice_commands=TSpeechObject(
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

else:
    def walk (*args, **kwargs):
        return []
    
    def get_voice_default_config() -> Optional[VoiceConfig]:
        return None
