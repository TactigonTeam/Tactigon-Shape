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

from flask import render_template, redirect, url_for, flash
from flask.blueprints import Blueprint

from tactigon_shapes.modules.file_manager.manager import get_file_manager_extension
from tactigon_shapes.utils.request_utils import get_from_request

bp = Blueprint('file_manager', __name__, url_prefix="/files",template_folder='templates', static_folder='static')

@bp.route("/")
@bp.route("/<string:folder>")
def index(folder: str | None = None):
    app = get_file_manager_extension()

    if not app:
        flash("File Manager extension not initialized", "danger")
        return redirect(url_for('main.index'))
    
    folders = app.get_directories()

    return render_template('file_manager/index.jinja', folders=folders, current_directory=folder)


@bp.route("folder/add", methods=["POST"])
def add_folder():
    app = get_file_manager_extension()

    if not app:
        flash("File Manager extension not initialized", "danger")
        return redirect(url_for('main.index'))
    
    folder_name = get_from_request("folder_name")

    if not folder_name:
        flash("Folder name is required", "danger")
        return redirect(url_for('file_manager.index'))
    
    app.add_directory(folder_name)
    
    # Logic to add a new folder would go here
    return redirect(url_for('file_manager.index'))