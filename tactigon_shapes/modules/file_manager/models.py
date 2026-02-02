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

class ItemType(str, Enum):
    FILE = "file"
    FOLDER = "folder"
    DIRECTORY = "directory"

@dataclass
class ContentItem:
    item_type: ItemType

    @classmethod
    def FromJSON(cls, json: dict) -> "ContentItem":
        return cls(
            item_type=json["item_type"]
        )
    
    def toJSON(self) -> dict:
        return {
            "item_type": self.item_type,
        }

@dataclass
class DirectoryItem(ContentItem):
    name: str
    base_path: str
    item_type: ItemType = field(init=False, default=ItemType.DIRECTORY)

    @classmethod
    def FromJSON(cls, json: dict) -> "DirectoryItem":
        return cls(
            name=json.get("name", ""),
            base_path=json.get("base_path", ""),
        )
    
    def toJSON(self) -> dict:
        return {
            "item_type": self.item_type,
            "name": self.name,
            "base_path": self.base_path,
        }
    
@dataclass
class FolderItem(ContentItem):
    name: str
    base_path: str
    children: list[ContentItem] = field(default_factory=list)
    item_type: ItemType = field(init=False, default=ItemType.FOLDER)

    @classmethod
    def FromPath(cls, path: str):
        if path.count("/") < 2 :
            base_path = ""
            name = path.rsplit("/", 1)[0]
        else:
            base_path, name = path.rsplit("/", 1)
        return cls(
            name=name,
            base_path=f"/{base_path}"
        )

    @classmethod
    def FromJSON(cls, json: dict) -> "FolderItem":
        return cls(
            name=json.get("name", ""),
            base_path=json.get("base_path", ""),
        )
    
    def toJSON(self) -> dict:
        return {
            "item_type": self.item_type,
            "name": self.name,
            "base_path": self.base_path,
            "children": [c.toJSON() for c in self.children]
        }

@dataclass
class FileItem(ContentItem):
    name: str
    path: str
    size: int
    modified_time: datetime
    item_type: ItemType = field(init=False, default=ItemType.FILE)

    @classmethod
    def FromJSON(cls, json: dict) -> "FileItem":
        return cls(
            name=json.get("name", ""),
            path=json.get("path", ""),
            size=json.get("size", 0),
            modified_time=datetime.fromisoformat(json.get("modified_time", datetime.now().isoformat())),
        )
    
    def toJSON(self) -> dict:
        return {
            "item_type": self.item_type,
            "name": self.name,
            "path": self.path,
            "size": self.size,
            "modified_time": self.modified_time.isoformat(),
        }
    
class ItemBuilder:
    item_map = {
        ItemType.DIRECTORY: DirectoryItem,
        ItemType.FOLDER: FolderItem,
        ItemType.FILE: FileItem
    }
    
    @staticmethod
    def fromJSON(json: dict) -> ContentItem:
        item = ItemBuilder.item_map.get(json["item_type"], ContentItem)
        return item.FromJSON(json)

@dataclass
class FileManagerConfig:
    base_path: str
    directories: list[DirectoryItem] = field(default_factory=list)

    @classmethod
    def Default(cls, base_path) -> "FileManagerConfig":
        return cls(
            base_path="",
            directories=[
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