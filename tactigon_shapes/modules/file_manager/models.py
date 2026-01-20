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
from enum import Enum
from datetime import datetime

class ContentType(str, Enum):
    FILE = "file"
    FOLDER = "folder"
    DIRECTORY = "directory"

@dataclass
class ContentItem:
    content_type: ContentType

    @classmethod
    def FromJSON(cls, json: dict) -> "ContentItem":
        return cls(
            content_type=json["content_type"]
        )
    
    def toJSON(self) -> dict:
        return {
            "content_type": self.content_type,
        }

@dataclass
class DirectoryItem(ContentItem):
    name: str
    base_path: str
    content_type: ContentType = field(init=False, default=ContentType.DIRECTORY)

    @classmethod
    def FromJSON(cls, json: dict) -> "DirectoryItem":
        return cls(
            name=json.get("name", ""),
            base_path=json.get("base_path", ""),
        )
    
    def toJSON(self) -> dict:
        return {
            "content_type": self.content_type,
            "name": self.name,
            "base_path": self.base_path,
        }
    
@dataclass
class FolderItem(ContentItem):
    name: str
    base_path: str
    content_type: ContentType = field(init=False, default=ContentType.FOLDER)

    @classmethod
    def FromJSON(cls, json: dict) -> "FolderItem":
        return cls(
            name=json.get("name", ""),
            base_path=json.get("base_path", ""),
        )
    
    def toJSON(self) -> dict:
        return {
            "content_type": self.content_type,
            "name": self.name,
            "base_path": self.base_path,
        }

@dataclass
class FileItem(ContentItem):
    name: str
    size: int
    modified_time: datetime
    content_type: ContentType = field(init=False, default=ContentType.FILE)

    @classmethod
    def FromJSON(cls, json: dict) -> "FileItem":
        return cls(
            name=json.get("name", ""),
            size=json.get("size", 0),
            modified_time=datetime.fromisoformat(json.get("modified_time", datetime.now().isoformat())),
        )
    
    def toJSON(self) -> dict:
        return {
            "content_type": self.content_type,
            "name": self.name,
            "size": self.size,
            "modified_time": self.modified_time.isoformat(),
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
    
class ItemAlreadyExists(Exception):
    def __init__(self, message: str):
        self.message = message