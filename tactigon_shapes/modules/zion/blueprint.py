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


from flask import Blueprint, redirect, render_template, flash, url_for

from tactigon_shapes.modules.zion.manager import get_zion_interface
from tactigon_shapes.modules.zion.models import ZionConfig

from tactigon_shapes.config import app_config
from tactigon_shapes.utils.request_utils import get_from_request, stop_running_shape, stop_shape

bp = Blueprint("zion", __name__, url_prefix="/zion", template_folder="templates", static_folder="static")

@bp.before_request
def manage():
    stop_running_shape()

@bp.route("/")
def index(edit: bool = False):
    app = get_zion_interface()

    if not app:
        flash("Zion interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    if app.config and app.config.is_valid():
        config = app.config
    else:
        config = ZionConfig.Default()
        edit = True

    return render_template("zion/index.jinja", configured=app.configured, config=config, edit=edit, devices=app.devices)

@bp.route("/edit")
def edit():
    app = get_zion_interface()

    if not app:
        flash("Zion interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    return index(True)

@bp.route("/save", methods=["POST"])
def save():
    app = get_zion_interface()

    if not app:
        flash("Zion interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    username = get_from_request("username")
    password = get_from_request("password")
    url = get_from_request("url")

    if not username or not password:
        flash("Cannot save Zion configurations. Username or password are required", category="danger")
        return redirect(url_for("zion.index"))
    
    _url = url if url else ZionConfig.url

    token = app.refresh_token(_url, username, password)

    if not token:
        flash("Cannot save Zion configurations. Username or password or url are incorrect", category="danger")
        return redirect(url_for("zion.index"))
    
    new_config = ZionConfig(username, password, _url, token)
    app.save_config(new_config)

    flash("Zion configured succesfully", category="success")
    return redirect(url_for("zion.index"))

@bp.route("/remove")
def remove():
    app = get_zion_interface()

    if not app:
        flash("Zion interface not running", category="danger")
        return redirect(url_for("main.index"))
       
    app.reset_config()

    flash("Zion configuration removed", category="success")
    return redirect(url_for("zion.edit"))

@bp.route("/devices/refresh")
def refresh_devices():
    app = get_zion_interface()

    if not app:
        flash("Zion interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    app.devices = app.get_devices()

    flash(F"Zion device list refreshed ({len(app.devices)} loaded)", category="success")
    return redirect(url_for("zion.index"))