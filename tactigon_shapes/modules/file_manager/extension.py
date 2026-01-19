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

import logging
import os
import json

from flask import Flask
from tactigon_shapes.modules.file_manager.models import FileManagerConfig, DirectoryItem, FileItem
from tactigon_shapes.models import BASE_PATH

class FileManagerExtension:
    def __init__(self, config_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(FileManagerExtension.__name__)
        self.config_path = config_path

        self.load_config()

        if app:
            self.init_app(app)

    @property
    def config_file(self) -> str:
        return os.path.join(self.config_path, "config.json")

    def init_app(self, app: Flask):
        app.extensions[FileManagerExtension.__name__] = self

    def load_config(self):
        if os.path.exists(self.config_path) and os.path.isfile(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)
                self.config = FileManagerConfig.FromJSON(config_data)
                self._logger.info("FileManager configuration loaded")
        else:
            self.config = FileManagerConfig.Default(BASE_PATH)
            self.save_config()

    def save_config(self):
        if not os.path.exists(self.config_path):
            os.makedirs(self.config_path)

        with open(self.config_file, "w") as f:
            json.dump(self.config.toJSON(), f, indent=2)

        self._logger.info("FileManager configuration saved")
        self.load_config()

    def get_directories(self) -> list[DirectoryItem]:
        return self.config.directories
    
    def add_directory(self, name: str):
        new_directory = DirectoryItem(name=name, base_path=os.path.join(BASE_PATH, name))

        self.config.directories.append(new_directory)
        self.save_config()