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

from flask import render_template, redirect, url_for, flash, request, send_file
from flask.blueprints import Blueprint

from tactigon_shapes.modules.file_manager.extension import FileManagerExtension
from tactigon_shapes.modules.file_manager.manager import get_file_manager_extension
from tactigon_shapes.modules.file_manager.models import ItemAlreadyExists, ItemBuilder
from tactigon_shapes.utils.request_utils import get_from_request

bp = Blueprint('file_manager', __name__, url_prefix="/files",template_folder='templates', static_folder='static')

@bp.route("/")
def index():
    app = get_file_manager_extension()

    if not app:
        flash("File Manager extension not initialized", "danger")
        return redirect(url_for('main.index'))

    return render_template('file_manager/index.jinja')

@bp.route("directories/")
def list_directories():
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")

    directories = app.get_directories()

    return FileManagerExtension.build_success_response({"directories": [folder.toJSON() for folder in directories]})

@bp.route("directories/add", methods=["POST"])
def add_directories():
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")
    
    directory_name = get_from_request("directory_name")

    if not directory_name:
        return FileManagerExtension.build_error_response("Folder name is required")
    
    try:
        directory = app.add_directory(directory_name)
    except ItemAlreadyExists as e:
        return FileManagerExtension.build_error_response(e.message)
    except Exception as e:
        return FileManagerExtension.build_error_response(str(e))
    
    return FileManagerExtension.build_success_response({"directory": directory.toJSON()})

@bp.route("directories/<string:directory_name>/delete", methods=["POST"])
def delete_directory(directory_name: str):
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")
    
    directories = app.get_directories()
    directory = next((d for d in directories if d.name == directory_name), None)

    if not directory:
        return FileManagerExtension.build_error_response("Directory not found")
    
    app.delete_directory(directory)
    
    return FileManagerExtension.build_success_response()

@bp.route("directories/<string:directory_name>/content")
def list_directory_content(directory_name: str):
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")
    
    directories = app.get_directories()
    directory = next((d for d in directories if d.name == directory_name), None)

    if not directory:
        return FileManagerExtension.build_error_response("Directory not found")
    
    subfolders = get_from_request("subfolders")

    if not subfolders:
        subfolders = []
    else:
        subfolders = subfolders.split(",")
    
    contents = app.list_contents(directory, subfolders)

    return FileManagerExtension.build_success_response({"items": [item.toJSON() for item in contents]})

@bp.route("directories/<string:directory_name>/folders/add", methods=["POST"])
def add_folder_to_directory(directory_name: str):
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")
    
    directories = app.get_directories()
    directory = next((d for d in directories if d.name == directory_name), None)

    if not directory:
        return FileManagerExtension.build_error_response("Directory not found")
    
    folder_path = get_from_request("folder_path") or ""
    folder_name = get_from_request("folder_name")

    if not folder_name:
        return FileManagerExtension.build_error_response("Must specify a folder name.")
    
    try:
        folder = app.add_folder(directory, folder_path, folder_name)
    except ItemAlreadyExists as e:
        return FileManagerExtension.build_error_response(e.message)
    
    return FileManagerExtension.build_success_response({"folder": folder})

@bp.route("directories/<string:directory_name>/files/add", methods=["POST"])
def add_file_to_directory(directory_name: str):
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")
    
    directories = app.get_directories()
    directory = next((d for d in directories if d.name == directory_name), None)

    if not directory:
        return FileManagerExtension.build_error_response("Directory not found")
    
    folder_path = get_from_request("folder_path") or ""
    upload_file = request.files['file']

    if not upload_file:
        return FileManagerExtension.build_error_response("Must specify a folder name.")
    
    try:
        folder = app.add_file(directory, folder_path, upload_file)
    except ItemAlreadyExists as e:
        return FileManagerExtension.build_error_response(e.message)
    
    return FileManagerExtension.build_success_response({"folder": folder})

@bp.route("directories/<string:directory_name>/files/delete", methods=["DELETE"])
def delete_file_from_directory(directory_name: str):
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response("File Manager extension not initialized")
    
    if not request.is_json:
        return FileManagerExtension.build_error_response(
            "Request body must be JSON"
        )
    
    directories = app.get_directories()
    directory = next((d for d in directories if d.name == directory_name), None)

    if not directory:
        return FileManagerExtension.build_error_response("Directory not found")
    
    data = request.get_json(silent=True)

    if not data:
        return FileManagerExtension.build_error_response(
            "Invalid or empty JSON body"
        )
    
    items_to_delete = data.get("items", [])
   
    try:
        app.delete_items(directory, [ItemBuilder.fromJSON(i) for i in items_to_delete])
    except ItemAlreadyExists as e:
        return FileManagerExtension.build_error_response(e.message)
    
    return FileManagerExtension.build_success_response()

@bp.route("directories/<string:directory_name>/files/download", methods=["POST"])
def download_file_from_directory(directory_name: str):
    app = get_file_manager_extension()

    if not app:
        return FileManagerExtension.build_error_response(
            "File Manager extension not initialized"
        )

    if not request.is_json:
        return FileManagerExtension.build_error_response(
            "Request body must be JSON"
        )
    
    directories = app.get_directories()
    directory = next((d for d in directories if d.name == directory_name), None)

    if not directory:
        return FileManagerExtension.build_error_response("Directory not found")
    
    data = request.get_json(silent=True)

    if not data:
        return FileManagerExtension.build_error_response(
            "Invalid or empty JSON body"
        )
    
    items_to_download = data.get("items", [])

    if not items_to_download:
        return FileManagerExtension.build_error_response(
            "Must specify a list of item to download"
        )
    
    print(items_to_download)
    
    if len(items_to_download) == 1:
        result = app.download_item(ItemBuilder.fromJSON(items_to_download[0]))
    else:
        result = app.download_items([ItemBuilder.fromJSON(i) for i in items_to_download])

    if not result:
        return FileManagerExtension.build_error_response(
            "Item(s) not found"
        )

    file_to_send, mimetype, download_name = result

    return send_file(
        file_to_send,
        mimetype=mimetype,
        as_attachment=True,
        download_name=download_name
    )