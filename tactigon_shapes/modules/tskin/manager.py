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
from typing import Optional, Union

from tactigon_shapes.modules.tskin.models import TSkin, TSkinConfig, GestureConfig, TSkinModel, Hand

TSKIN_EXTENSION = "tskin"

def load_tskin(config: TSkinConfig):
    current_app.extensions[TSKIN_EXTENSION] = TSkin(config)

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
        )
    )