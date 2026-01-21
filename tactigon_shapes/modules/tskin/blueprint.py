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


import sys
import json
import asyncio
from os import path
from bleak import BleakScanner
from flask import Blueprint, render_template, redirect, url_for, request, flash, current_app

from tactigon_shapes.modules.tskin.manager import get_tskin_default_config, start_tskin, load_tskin, stop_tskin, get_tskin
from tactigon_shapes.modules.tskin.models import TSkinModel, TSkinConfig, Hand, GestureConfig, SocketConfig, TSpeechObject

from tactigon_shapes.modules.socketio import get_socket_app
from tactigon_shapes.config import app_config, speechs_file_path
from tactigon_shapes.utils.extensions import stop_apps
from tactigon_shapes.utils.request_utils import get_from_request

bp = Blueprint('tskin', __name__, url_prefix="/tskin", template_folder="templates")

@bp.before_request
def manage():
    stop_apps()

@bp.route("add", methods=["GET"])
def add():
    return render_template("tskin/add.jinja")

@bp.route("socket_settings", methods=["GET", "POST"])
def socket_settings():
    if request.method == "POST":
        host = get_from_request("host")
        port = get_from_request("port")
        ping = get_from_request("ping")

        if not host or not port or not ping:
            flash("Invalid configuration. Please check", "danger")
            return redirect(url_for("tskin.socket_settings"))
        
        port = int(port)
        ping = int(ping)

        config = SocketConfig(
            host=host,
            port=port,
            ping=ping,
        )

        app_config.TSKIN_SOCKET = config
        app_config.save()

        socket_app = get_socket_app()
        if socket_app:
            socket_app.stop()

        stop_tskin()

        if app_config.TSKIN and app_config.TSKIN_SOCKET:
            load_tskin(app_config.TSKIN, app_config.TSKIN_SOCKET)
            tskin = start_tskin()

            if socket_app and tskin:
                socket_app.setTSkin(tskin)

        flash("Socket saved!", "success")

    if not app_config.TSKIN_SOCKET:
        config = SocketConfig("0.0.0.0")
    else:
        config = app_config.TSKIN_SOCKET

    return render_template("tskin/socket_settings.jinja", config=config)

@bp.route("speech_settings", methods=["GET", "POST"])
def speech_settings():
    if request.method == "POST":
        scorer = get_from_request("scorer")

        if not scorer:
            flash("Please select a valid scorer", "danger")
            return redirect(url_for("tskin.speech_settings"))
        
        scorer_config = next((s for s in app_config.SCORERS if scorer == s.name), None)

        if not scorer_config:
            flash("Scorer not found in config", "danger")
            return redirect(url_for("tskin.speech_settings"))
        
        with open(path.join(speechs_file_path, scorer_config.speech_file)) as speech_file:
            app_config.TSKIN_SPEECH = TSpeechObject.FromJSON(json.load(speech_file))
        app_config.SELECTED_SCORER = scorer

        app_config.save()

        flash("Speech saved!", "success")
       
    return render_template("tskin/speech_settings.jinja", scorers=app_config.SCORERS, selected_scorer=app_config.SELECTED_SCORER)

@bp.route("scan", methods=["GET"])
def scan():
    async def find_devices():
        devices = await BleakScanner.discover()
        return filter(lambda d: str(d.name).startswith("TSKIN") , devices)

    devices = asyncio.run(find_devices())
    tskins = [{"name": d.name, "id": d.address, "address": d.address} for d in devices]

    return json.dumps(tskins)

@bp.route("hand", methods=["POST"])
def hand():
    tskin_name = get_from_request("tskin_name")
    tskin_mac = get_from_request("tskin_mac")

    if not (tskin_mac and tskin_name):
        flash("You need to select a Tactigon Skin to proceed", category="danger")
        return redirect(url_for("tskin.add"))

    return render_template("tskin/hand.jinja", tskin_name=tskin_name, tskin_mac=tskin_mac)

@bp.route("model", methods=["POST"])
def model():
    tskin_name = get_from_request("tskin_name")
    tskin_mac = get_from_request("tskin_mac")

    if not tskin_name or not tskin_mac:
        return redirect(url_for("tskin.add"))

    hand = get_from_request("hand")

    if not hand:
        flash("Please select a hand", "danger")
        return redirect(url_for("tskin.add"))

    models = list(filter(lambda m: m.hand.value==hand, app_config.MODELS))

    if len(models) == 1:
        return redirect(url_for("tskin.save", tskin_name=tskin_name, tskin_mac=tskin_mac, hand=hand, model=models[0].name), code=307)

    return render_template("tskin/model.jinja", tskin_name=tskin_name, tskin_mac=tskin_mac, hand=hand, models=models)

@bp.route("/remove", methods=["GET"])
def remove():
    socket_app = get_socket_app()

    tskin = get_tskin()

    if tskin is None:
        # TODO: Handle case where TSKIN object does not exists but has somewhat valid configuration
        

        flash(F"Tactigon skin {hand} not configured!", "danger")
        return redirect(url_for("main.index"))

    if socket_app:
        socket_app.stop()
    
    stop_tskin()

    app_config.TSKIN = None
    
    app_config.save()
    current_app.config.from_object(app_config)

    if socket_app:
        socket_app.init_app(current_app)

    return redirect(url_for("main.index"))

@bp.route("save", methods=["POST"])
def save():
    socket_app = get_socket_app()

    tskin_name = get_from_request("name")
    tskin_mac = get_from_request("mac")

    if not tskin_name or not tskin_mac:
        flash("Please select a Tactigon Skin", "danger")
        return {"error": "name or mac error"}, 500

    hand = get_from_request("hand")
    
    if not hand:
        flash("Please select a hand", "danger")
        return {"error": "hand error"}, 500

    # if not model_name:
    #     flash("Please select a model", "danger")
    #     return redirect(url_for("tskin.add", tskin_name=tskin_name, tskin_mac=tskin_mac, hand=hand), code=307)
        
    hand = Hand(hand)
    model_name = "MODEL_01_" + hand.name

    current_model: TSkinModel | None = None

    for model in app_config.MODELS:
        if model.name == model_name:
            current_model = model
            break

    if current_model is None:
        flash("Error MODEL", "danger")
        return {"error": "model error"}, 500
        # return redirect(url_for("tskin.add"))
    
    tskin_config, socket_config, tspeech_object = get_tskin_default_config(tskin_mac, hand, tskin_name, current_model)

    if socket_app:
        socket_app.stop()

    stop_tskin()
    app_config.TSKIN = tskin_config
    app_config.TSKIN_SOCKET = socket_config
    app_config.TSKIN_SPEECH = tspeech_object

    app_config.save()
    current_app.config.from_object(app_config)

    load_tskin(tskin_config, socket_config)
    tskin = start_tskin()

    if socket_app and tskin:
        socket_app.setTSkin(tskin)

    return json.dumps(tskin_config.toJSON()), 200

@bp.route("play", methods=["POST"])
def play(base_path = ""):
    tskin = get_tskin()

    if not tskin:
        return "Error: TSkin not present"
    
    data = request.get_json(force=True)

    if not data:
        return "Error: No JSON in request"
    
    audio_file = data.get("audio_file")

    if not audio_file:
        return "Error: No audio_file in JSON"
    
    base_path = data.get("base_path")

    if base_path:
        audio_file = path.join(base_path, audio_file)

    # tskin.play(audio_file)
    # TODO: implement play

    return "Ok"