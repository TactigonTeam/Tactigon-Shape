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
import json

from os import path, makedirs, remove
from typing import TypeVar
from flask import Flask, current_app
from abc import ABC, abstractmethod


class InterfaceConfig(ABC):
    @abstractmethod
    def toJSON(self) -> dict:
        pass

    @classmethod
    @abstractmethod
    def FromJSON(cls, data: dict):
        pass

    @classmethod
    def Default(cls):
        return None


class Interface(ABC):
    _logger: logging.Logger
    _config_path: str
    _config: InterfaceConfig | None = None

    config_class: type[InterfaceConfig] | None = None

    def __init__(self, config_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(self.name)
        self._config_path = config_path
        self.load_config()
        self.post_init()
        if app:
            self.init_app(app)

    @property
    @abstractmethod
    def running(self) -> bool:
        pass

    @property
    def config_file(self) -> str:
        return path.join(self._config_path, "config.json")
    
    @property
    def name(self) -> str:
        return type(self).__name__

    @property
    def configured(self) -> bool:
        return False if self._config is None else True

    @property
    def can_run(self) -> bool:
        return self.configured

    def post_init(self):
        pass

    def post_load_config(self):
        pass

    def init_app(self, flask_app: Flask):
        flask_app.extensions[self.name] = self

    def load_config(self):
        if not self.config_class:
            self._logger.error("Config class not set for %s", self.name)
            raise ValueError("Config class not set for {}".format(self.name))
        
        if path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                self._config = self.config_class.FromJSON(json.load(f))
                self._logger.debug("%s configuration loaded. %s", self.name, self._config)
        else:
            self._config = None

        self.post_load_config()

    def save_config(self, config: InterfaceConfig):
        if not path.exists(self._config_path):
            makedirs(self._config_path)

        with open(self.config_file, "w") as f:
            json.dump(config.toJSON(), f)

        self._logger.info("%s configuration saved.", self.name)
        self.load_config()

    def reset_config(self):
        """ remove config file and reloads it
        """
        if path.exists(self._config_path) and path.exists(self.config_file):
            remove(self.config_file)

        self.load_config()


T = TypeVar("T", bound=Interface)


def get_interface(interface: type[T], app: Flask = current_app) -> T | None:
    ext = app.extensions.get(interface.__name__)
    if isinstance(ext, interface):
        return ext
    return None