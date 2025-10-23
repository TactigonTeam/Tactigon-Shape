#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Contributors:
# Massimiliano Bellino
# Stefano Barbareschi
#********************************************************************************/


from flask import current_app

from typing import Optional

from ..modules.socketio import SocketApp
from ..modules.braccio.extension import BraccioInterface
from ..modules.zion.extension import ZionInterface
from ..modules.tskin.manager import TSKIN_EXTENSION
from ..modules.ironboy.extension import IronBoyInterface

excluded_apps = [SocketApp.name, TSKIN_EXTENSION, "socketio", BraccioInterface.__name__, ZionInterface.__name__,IronBoyInterface.__name__]

def stop_apps(exclude: Optional[str] = None):
    l = excluded_apps
    if (exclude):
        l.append(exclude)

    for ext in current_app.extensions:
        if ext in l:
            continue
        current_app.extensions[ext].stop()