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


from functools import wraps
from flask import Blueprint, render_template, flash, redirect, url_for, current_app

from tactigon_shapes import __version__

from tactigon_shapes.config import check_config
from tactigon_shapes.modules.socketio import get_socket_app
from tactigon_shapes.utils.extensions import stop_apps

from tactigon_shapes.modules.tskin.manager import stop_tskin

bp = Blueprint('main', __name__, template_folder="main")

@bp.route("/", methods=["GET"])
@check_config
def index():
    return redirect(url_for("shapes.index"))

@bp.route("/settings")
def settings():
    return render_template("info.jinja", version=__version__)