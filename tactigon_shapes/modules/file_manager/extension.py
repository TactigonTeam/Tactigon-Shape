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
import shutil
import io
import mimetypes
import zipfile

from datetime import datetime
from flask import Flask
from pathlib import Path
from werkzeug.datastructures import FileStorage

from tactigon_shapes.interfaces.interface import Interface
from tactigon_shapes.modules.file_manager.models import FileManagerConfig, ContentItem, DirectoryItem, FolderItem, FileItem, ItemAlreadyExists
from tactigon_shapes.models import BASE_PATH

EXCLUDED_FILES = [".gitkeep"]

class FileManager(Interface):
    _config: FileManagerConfig | None = None

    config_class = FileManagerConfig

    @property
    def running(self) -> bool:
        return self._config is not None
    
    @staticmethod
    def get_logger() -> logging.Logger:
        return logging.getLogger(FileManager.__name__)

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
    
    @staticmethod
    def get_file_extension(filename: str) -> str:
        p = Path(filename)
        return "".join(p.suffixes)
    
    @staticmethod
    def list_folders(directory: DirectoryItem) -> list[FolderItem]:
        if os.path.exists(directory.base_path):
            folder_list = [
                FolderItem.FromPath(f"{root.replace(directory.base_path, '')}")
                for root, *_ in os.walk(directory.base_path)
                if "__pycache__" not in root
            ]
            FileManager.get_logger().info("Got %s folders from %s", len(folder_list), directory.base_path)
            return folder_list
        
        FileManager.get_logger().info("Directory %s does not exists", directory.base_path)
        return []
    
    @staticmethod
    def list_files(directory: DirectoryItem, folder: FolderItem | None = None) -> list[FileItem]:
        if folder:
            full_path = os.path.join(directory.base_path, folder.base_path)
        else:
            full_path = directory.base_path

        if os.path.exists(directory.base_path) and os.path.isdir(directory.base_path):
            file_list = [
                FileItem(name=entry.name, path=entry.path, size=entry.stat().st_size, modified_time=datetime.now())
                for entry in os.scandir(full_path)
                if entry.is_file() and entry.name not in EXCLUDED_FILES
            ]
            FileManager.get_logger().info(f"Listed {len(file_list)} files in path '{full_path}'")
            return file_list

        FileManager.get_logger().info("Path %s does not exists", full_path)
        return []
    
    @staticmethod
    def list_contents(directory: DirectoryItem, subfolders: list[str] | None = None, recursive: bool = False) -> list[ContentItem]: 
        if subfolders is None:
            subfolders = []
        _logger = FileManager.get_logger()
        contents: list[ContentItem] = []
        content_path = os.path.join(directory.base_path, *subfolders)
        _logger.info(f"Listing contents in path: {content_path}")
        if os.path.exists(content_path) and os.path.isdir(content_path):
            for entry in os.scandir(content_path):
                if entry.is_file():
                    if entry.name in EXCLUDED_FILES:
                        continue
                    
                    file_item = FileItem(
                        name=entry.name,
                        path=entry.path,
                        size=entry.stat().st_size,
                        modified_time=datetime.fromtimestamp(entry.stat().st_mtime)
                    )
                    contents.append(file_item)
                elif entry.is_dir():
                    if recursive:
                        subfolders.append(entry.name)
                        contents.extend(FileManager.list_contents(directory, subfolders, recursive))
                    else:
                        dir_item = FolderItem(
                            name=entry.name,
                            base_path=entry.path
                        )
                        contents.append(dir_item)

        _logger.info(f"Listed contents in directory '{content_path}': {len(contents)} items")
        return contents
    
    @staticmethod
    def add_directory(base_path: str,name: str) -> DirectoryItem:
        new_directory = DirectoryItem(name=name, base_path=os.path.join(base_path, name))

        if os.path.exists(new_directory.base_path):
            raise ItemAlreadyExists("Directory already exists")
        
        os.makedirs(new_directory.base_path)
        FileManager.get_logger().info(f"Directory '{name}' added")
        return new_directory
    
    @staticmethod
    def add_folder(directory: DirectoryItem, folder_path: str, folder: str) -> FolderItem:
        _logger = FileManager.get_logger()
        if folder_path.endswith("/"):
            folder_path = folder_path[:-1]

        new_folder = FolderItem(name=folder, base_path=os.path.join(directory.base_path, folder_path, folder))

        if os.path.exists(new_folder.base_path):
            _logger.error("Folder already exists. %s", new_folder.base_path)
            raise ItemAlreadyExists("Folder already exists")
        
        os.makedirs(new_folder.base_path)
        _logger.info(f"Created folder '{new_folder}'")
        return new_folder
    
    @staticmethod
    def add_file(directory: DirectoryItem, file_path: str, content: FileStorage) -> FileItem:
        _logger = FileManager.get_logger()

        if file_path.endswith("/"):
            file_path = file_path[:-1]

        full_path = os.path.join(directory.base_path, file_path)

        _logger.info("Adding file %s", full_path)
    
        if not os.path.exists(full_path):
            _logger.info("Path does not exists. Creating path...")
            os.makedirs(full_path)

        file_full_path = os.path.join(full_path, content.filename or ".temp")
        if os.path.exists(file_full_path):
            raise ItemAlreadyExists(f"File {content.filename} already exsists")

        content.save(file_full_path)
        
        _logger.info(f"File '{content.filename}' added to '{full_path}'")
        return FileItem(
            name=content.filename or ".temp",
            path=file_full_path,
            size=content.content_length,
            modified_time=datetime.now()
        )
    
    @staticmethod
    def delete_items(directory: DirectoryItem, items: list[ContentItem]):
        _logger = FileManager.get_logger()
        _logger.info("Removing %s items", len(items))

        for item in items:
            if isinstance(item, FileItem):
                FileManager.delete_file_item(directory, item)
            elif isinstance(item, FolderItem):
                FileManager.delete_folder_item(directory, item)
            elif isinstance(item, DirectoryItem):
                FileManager.delete_directory_item(item)

    @staticmethod
    def delete_file_item(directory: DirectoryItem, item: FileItem):
        file_path = os.path.join(directory.base_path, item.name)
        if os.path.exists(file_path):
            os.remove(file_path)
            FileManager.get_logger().info(f"File '{file_path}' deleted")

    @staticmethod
    def delete_folder_item(directory: DirectoryItem, item: FolderItem | DirectoryItem):
        folder_path = os.path.join(directory.base_path, item.name)
        if os.path.exists(folder_path):
            FileManager.get_logger().info(f"Folder '{folder_path}' deleted")
            shutil.rmtree(folder_path)

    @staticmethod
    def delete_directory_item(item: DirectoryItem):
        folder_path = os.path.join(item.base_path)
        if os.path.exists(folder_path):
            FileManager.get_logger().info(f"Directory '{folder_path}' deleted")
            shutil.rmtree(folder_path)
    
    @staticmethod
    def download_file(file_item: FileItem) -> tuple[io.BytesIO, str, str] | None:
        _logger = FileManager.get_logger()
        if os.path.exists(file_item.path) and os.path.isfile(file_item.path):
            _logger.info("Downloading file %s", file_item.name)
            file_buffer = io.BytesIO()
            with open(file_item.path, "rb") as f:
                file_buffer.write(f.read())
            file_buffer.seek(0)
            return (file_buffer, mimetypes.guess_extension(file_item.name) or "", file_item.name)
        else:
            _logger.error("Cannot download file %s with path %s", file_item.name, file_item.path)
            return None
    
    @staticmethod
    def download_folder(folder_item: FolderItem) -> tuple[io.BytesIO, str, str] | None:
        _logger = FileManager.get_logger()
        if os.path.exists(folder_item.base_path) and os.path.isdir(folder_item.base_path):
            _logger.info("Downloading folder %s and it's content", folder_item.name)
            zip_buffer = io.BytesIO()

            with zipfile.ZipFile(zip_buffer, mode="w", compression=zipfile.ZIP_DEFLATED) as zipf:
                for root, dirs, files in os.walk(folder_item.base_path):
                    for file in files:
                        full_path = os.path.join(root, file)
                        arcname = os.path.relpath(full_path, folder_item.base_path)
                        arcname = os.path.join(folder_item.name, arcname)
                        zipf.write(full_path, arcname=arcname)


            zip_buffer.seek(0)
            return zip_buffer, "application/zip", f"{folder_item.name}.zip"
    
        else:
            _logger.error("Cannot download folder %s with path %s", folder_item.name, folder_item.base_path)
            return None

    @staticmethod
    def download_item(item: ContentItem) -> tuple[io.BytesIO, str, str] | None:
        if isinstance(item, FileItem):
            return FileManager.download_file(item)
        elif isinstance(item, FolderItem):
            return FileManager.download_folder(item)
        
        FileManager.get_logger().warning("Invalid item to download %s", item.toJSON())
        return None
    
    @staticmethod
    def download_items(items: list[ContentItem]) -> tuple[io.BytesIO, str, str] | None:
        _logger = FileManager.get_logger()
        item_count = 0
        zip_buffer = io.BytesIO()

        with zipfile.ZipFile(zip_buffer, mode="w", compression=zipfile.ZIP_DEFLATED) as zipf:
            for item in items:
                if isinstance(item, FileItem):
                    if os.path.exists(item.path) and os.path.isfile(item.path):
                        item_count += 1
                        zipf.write(item.path, arcname=item.name)
                elif isinstance(item, FolderItem):
                    if os.path.exists(item.base_path) and os.path.isdir(item.base_path):
                        for root, dirs, files in os.walk(item.base_path):
                            for file in files:
                                item_count += 1
                                full_path = os.path.join(root, file)
                                arcname = os.path.relpath(full_path, item.base_path)
                                arcname = os.path.join(item.name, arcname)
                                zipf.write(full_path, arcname=arcname)

        zip_buffer.seek(0)

        if item_count == 0:
            _logger.warning("No items to download...")
            return None
        
        _logger.info("Downloading %s items", item_count)

        return zip_buffer, "application/zip", "download.zip"

    def post_load_config(self):
        if self._config is None:
            self.save_config(FileManagerConfig.Default(BASE_PATH))

    def get_directories_from_config(self) -> list[DirectoryItem]:
        if self._config is None:
            raise ValueError("Config is not loaded")
        
        return self._config.directories
    
    def add_directory_to_config(self, directory: DirectoryItem):
        if self._config is None:
            raise ValueError("Config is not loaded")
        
        self._config.directories.append(directory)
        self.save_config(self._config)
        self._logger.info(f"Directory '{directory.name}' added to config")
    
    def remove_directory_from_config(self, directory: DirectoryItem):
        if self._config is None:
            raise ValueError("Config is not loaded")

        self._config.directories = [d for d in self._config.directories if d.name != directory.name]
        self.save_config(self._config)
        self._logger.info(f"Directory '{directory.name}' deleted")

    def get_shape_blocks(self):
        if self._config is None:
            raise ValueError("Config is not loaded")
        
        blocks = []
        for directory in self._config.directories:
            blocks.append({
                "directory": directory.toJSON(),
                "content": [c.toJSON() for c in self.list_contents(directory, recursive=True)]
            })

        return blocks