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

from os import path
from dataclasses import dataclass, field


@dataclass
class DirectoryItem:
    name: str
    base_path: str

    @classmethod
    def FromJSON(cls, json: dict) -> "DirectoryItem":
        return cls(
            name=json.get("name", ""),
            base_path=json.get("base_path", ""),
        )
    
    def toJSON(self) -> dict:
        return {
            "name": self.name,
            "base_path": self.base_path,
        }

@dataclass
class FileItem:
    name: str
    size: int
    modified_time: str

    @classmethod
    def FromJSON(cls, json: dict) -> "FileItem":
        return cls(
            name=json.get("name", ""),
            size=json.get("size", 0),
            modified_time=json.get("modified_time", ""),
        )
    
    def toJSON(self) -> dict:
        return {
            "name": self.name,
            "size": self.size,
            "modified_time": self.modified_time,
        }   

@dataclass
class FileManagerConfig:
    base_path: str
    directories: list[DirectoryItem] = field(default_factory=list)

    @classmethod
    def Default(cls, base_path) -> "FileManagerConfig":
        return cls(
            base_path="",
            directories=[
                DirectoryItem(name="Shapes", base_path=path.join(base_path, "shapes")),
                DirectoryItem(name="DataFrame", base_path=path.join(base_path, "DataFrame")),
            ],
        )

    @classmethod
    def FromJSON(cls, json: dict) -> "FileManagerConfig":
        return cls(
            base_path=json.get("base_path", ""),
            directories=[DirectoryItem.FromJSON(dir) for dir in json.get("directories", [])],
        )
    
    def toJSON(self) -> dict:
        return {
            "base_path": self.base_path,
            "directories": [dir_item.toJSON() for dir_item in self.directories],
        }