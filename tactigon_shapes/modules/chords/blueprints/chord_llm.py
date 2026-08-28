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


from tabnanny import check
from flask import Blueprint, redirect, render_template, flash, url_for

from tactigon_shapes.modules.chords.manager import get_chord_llm_interface
from tactigon_shapes.modules.chords.models import ChordLLMConfig

from tactigon_shapes.config import app_config, check_config
from tactigon_shapes.utils.request_utils import get_from_request

bp = Blueprint("chord_llm", __name__, url_prefix="/chord-llm", template_folder="../templates")

@bp.route("/")
@check_config
def index(edit: bool = False):
    app = get_chord_llm_interface()

    if not app:
        flash("Chords interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    if app.config and app.config.is_valid():
        config = app.config
    else:
        config = ChordLLMConfig.Default()
        edit = True
        
    return render_template("chord_llm/index.jinja", configured=app.configured, config=config, edit=edit, prompts=app.prompts)

@bp.route("/edit")
@check_config
def edit():
    app = get_chord_llm_interface()

    if not app:
        flash("Chords interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    return index(True)

@bp.route("/save", methods=["POST"])
@check_config
def save():
    app = get_chord_llm_interface()

    if not app:
        flash("Chords interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    username = get_from_request("username")
    password = get_from_request("password")
    url = get_from_request("url")

    if not username or not password:
        flash("Cannot save Chords configurations. Username or password are required", category="danger")
        return redirect(url_for("chord_llm.index"))
    
    _url = url if url else ChordLLMConfig.url

    token = app.login(username, password)

    if not token:
        flash("Cannot save Chords configurations. Username or password or url are incorrect", category="danger")
        return redirect(url_for("chord_llm.index"))
    
    new_config = ChordLLMConfig(
        username=username, 
        password=password, 
        url=_url
    )

    app.save_config(new_config)

    flash("Chords configured succesfully", category="success")
    return redirect(url_for("chord_llm.index"))


@bp.route("/remove")
@check_config
def remove():
    app = get_chord_llm_interface()

    if not app:
        flash("Chords interface not running", category="danger")
        return redirect(url_for("main.index"))
       
    app.reset_config()

    flash("Chords configuration removed", category="success")
    return redirect(url_for("chord_llm.edit"))


@bp.route("/prompts/refresh")
@check_config
def refresh_prompts():
    app = get_chord_llm_interface()

    if not app:
        flash("Chords interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    app.prompts = app.get_prompts()

    flash(F"Chords prompt list refreshed ({len(app.prompts)} loaded)", category="success")
    return redirect(url_for("chord_llm.index"))