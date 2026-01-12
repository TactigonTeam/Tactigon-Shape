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
from os import path
from flask import current_app

from tactigon_shapes.modules.tskin.models import TSkin, TSkinConfig, GestureConfig, SocketConfig, TSkinModel, Hand, TSpeech, TSpeechObject, HotWord

TSKIN_EXTENSION = "tskin"

def load_tskin(config: TSkinConfig, socket_config: SocketConfig):
    current_app.extensions[TSKIN_EXTENSION] = TSkin(config, socket_config)

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

def get_tskin() -> TSkin | None:
    if TSKIN_EXTENSION in current_app.extensions:
        if isinstance(current_app.extensions[TSKIN_EXTENSION], TSkin):
            return current_app.extensions[TSKIN_EXTENSION]

    return None

def reset_tskin():
    current_app.extensions[TSKIN_EXTENSION] = None

def get_tskin_default_config(address: str, hand: Hand, name: str, model: TSkinModel) -> tuple[TSkinConfig, SocketConfig, TSpeechObject]:
    return (
        TSkinConfig(
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
        SocketConfig(
            host="0.0.0.0"
        ),
        TSpeechObject(
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