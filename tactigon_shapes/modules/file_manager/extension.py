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

from datetime import datetime
from flask import Flask
from werkzeug.datastructures import FileStorage
from tactigon_shapes.modules.file_manager.models import FileManagerConfig, ContentItem, DirectoryItem, FolderItem, FileItem, ItemAlreadyExists
from tactigon_shapes.models import BASE_PATH

class FileManagerExtension:
    def __init__(self, config_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(FileManagerExtension.__name__)
        self.config_path = config_path

        self.load_config()

        if app:
            self.init_app(app)

    @staticmethod
    def build_error_response(error: str) -> tuple[dict, int]:
        return {
            "success": False,
            "severity": "danger",
            "error": error
        }, 500
    
    @staticmethod
    def build_success_response(data: dict | None = None) -> tuple[dict, int]:
        response = {
            "success": True,
        }
        if data is not None:
            response.update(data)
        
        return response, 200

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
    
    def add_directory(self, name: str) -> DirectoryItem:
        new_directory = DirectoryItem(name=name, base_path=os.path.join(BASE_PATH, name))

        if os.path.exists(new_directory.base_path):
            raise ItemAlreadyExists("Directory already exists")
        
        os.makedirs(new_directory.base_path)
        self.config.directories.append(new_directory)
        self.save_config()
        self._logger.info(f"Directory '{name}' added")
        return new_directory
    
    def delete_directory(self, directory: DirectoryItem):
        self.config.directories = [d for d in self.config.directories if d.name != directory.name]
        self.save_config()
        self._logger.info(f"Directory '{directory.name}' deleted")

    def list_files(self, directory: DirectoryItem) -> list[FileItem]:
        files = []
        if os.path.exists(directory.base_path) and os.path.isdir(directory.base_path):
            for entry in os.scandir(directory.base_path):
                if entry.is_file():
                    file_item = FileItem(
                        name=entry.name,
                        size=entry.stat().st_size,
                        modified_time=datetime.now()
                    )
                    files.append(file_item)

        self._logger.info(f"Listed {len(files)} files in directory '{directory.name}'")
        return files
    
    def list_contents(self, directory: DirectoryItem, subfolders: list[str] = []) -> list[ContentItem]:
        contents: list[ContentItem] = []
        content_path = os.path.join(directory.base_path, *subfolders)
        self._logger.info(f"Listing contents in path: {content_path}")
        if os.path.exists(content_path) and os.path.isdir(content_path):
            for entry in os.scandir(content_path):
                if entry.is_file():
                    file_item = FileItem(
                        name=entry.name,
                        size=entry.stat().st_size,
                        modified_time=datetime.fromtimestamp(entry.stat().st_mtime)
                    )
                    contents.append(file_item)
                elif entry.is_dir():
                    dir_item = FolderItem(
                        name=entry.name,
                        base_path=entry.path
                    )
                    contents.append(dir_item)

        self._logger.info(f"Listed contents in directory '{content_path}': {len(contents)} items")
        return contents
    
    def add_folder(self, directory: DirectoryItem, folder_path: str, folder: str) -> FolderItem:
        if folder_path.endswith("/"):
            folder_path = folder_path[:-1]

        new_folder = FolderItem(name=folder, base_path=os.path.join(directory.base_path, folder_path, folder))

        if os.path.exists(new_folder.base_path):
            self._logger.error("Folder already exists. %s", new_folder.base_path)
            raise ItemAlreadyExists("Folder already exists")
        
        os.makedirs(new_folder.base_path)
        self._logger.info(f"Created folder '{new_folder}'")
        return new_folder

    def add_file(self, directory: DirectoryItem, file_path: str, content: FileStorage) -> FileItem:
        if file_path.endswith("/"):
            file_path = file_path[:-1]

        self._logger.info("directory %s file_path %s", directory.base_path, file_path)

        full_path = os.path.join(directory.base_path, file_path)
    
        if not os.path.exists(full_path):
            os.makedirs(full_path)

        file_path = os.path.join(full_path, content.filename or ".temp")
        if os.path.exists(file_path):
            raise ItemAlreadyExists(f"File {content.filename} already exsists")

        content.save(file_path)
        
        self._logger.info(f"File '{content.filename}' added to directory '{directory.name}'")
        return FileItem(
            name=content.filename or ".temp",
            size=content.content_length,
            modified_time=datetime.now()
        )

    def delete_file(self, directory: DirectoryItem, file_name: str):
        file_path = os.path.join(directory.base_path, file_name)
        if os.path.exists(file_path) and os.path.isfile(file_path):
            os.remove(file_path)
            self._logger.info(f"File '{file_name}' deleted from directory '{directory.name}'")
        else:
            self._logger.warning(f"File '{file_name}' not found in directory '{directory.name}'")

    def get_file(self, directory: DirectoryItem, file_name: str) -> bytes | None:    
        file_path = os.path.join(directory.base_path, file_name)
        if os.path.exists(file_path) and os.path.isfile(file_path):
            with open(file_path, "rb") as f:
                content = f.read()
            self._logger.info(f"File '{file_name}' retrieved from directory '{directory.name}'")
            return content
        else:
            self._logger.warning(f"File '{file_name}' not found in directory '{directory.name}'")
            return None