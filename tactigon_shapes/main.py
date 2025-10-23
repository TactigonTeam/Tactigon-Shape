#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Contributors:
# Massimiliano Bellino
# Stefano Barbareschi
#********************************************************************************/


from functools import wraps
from flask import Blueprint, render_template, flash, redirect, url_for, current_app

from . import __version__

from .config import check_config
from .modules.socketio import get_socket_app
from .utils.extensions import stop_apps

from .modules.tskin.manager import stop_tskin

bp = Blueprint('main', __name__, template_folder="main")

# @bp.before_request
# def manage():
#     socket_app = get_socket_app()
#     if socket_app:
#         socket_app.send_data(False)
#         socket_app.send_gesture(False)
#         socket_app.send_voice(False)

@bp.route("/", methods=["GET"])
@check_config
def index():
    return redirect(url_for("shapes.index"))

@bp.route("/settings")
def settings():
    return render_template("info.jinja", version=__version__)

@bp.route("/quit")
def quit():
    stop_apps()
    stop_tskin()

    app = get_socket_app()
    if app and app.is_running:
        app.stop()

    return "ok"