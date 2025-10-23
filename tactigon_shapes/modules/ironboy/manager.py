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


from flask import current_app

from typing import Optional

from .extension import IronBoyInterface

def get_ironboy_interface() -> Optional[IronBoyInterface]:
    if IronBoyInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[IronBoyInterface.__name__], IronBoyInterface):
        return current_app.extensions[IronBoyInterface.__name__]
    
    return None