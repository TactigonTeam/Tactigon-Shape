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


from flask import current_app

from tactigon_shapes.modules.file_manager.extension import FileManager

def get_file_manager_extension() -> FileManager | None:
    if FileManager.__name__ in current_app.extensions and isinstance(current_app.extensions[FileManager.__name__], FileManager):
        return current_app.extensions[FileManager.__name__]
    
    return None