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
from flask import Blueprint, render_template, redirect, url_for, request, flash

from tactigon_shapes.interfaces.interface import get_interface
from tactigon_shapes.modules.tskin.extension import DEFAULT_SCORER_CONFIG, DEFAULT_SOCKET_CONFIG, TSkinInterface
from tactigon_shapes.modules.tskin.models import Hand, SocketConfig

from tactigon_shapes.utils.extensions import stop_apps
from tactigon_shapes.utils.request_utils import get_from_request

bp = Blueprint('tskin', __name__, url_prefix="/tskin", template_folder="templates")

@bp.before_request
def manage():
    stop_apps()

@bp.route("add", methods=["GET"])
def add():
    print("TSKIN ADD", file=sys.stderr)
    return render_template("tskin/add.jinja")

@bp.route("socket_settings", methods=["GET", "POST"])
def socket_settings():
    tskin_interface = get_interface(TSkinInterface)

    if not tskin_interface:
        flash("TSkin interface not available", "danger")
        return redirect(url_for("main.settings"))
    
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

        tskin_interface.set_socket_config(config)
        tskin_interface.restart()

        flash("Socket saved!", "success")

    config = tskin_interface.get_socket_config() if tskin_interface else None
    if not config:
        config = SocketConfig("0.0.0.0")

    return render_template("tskin/socket_settings.jinja", config=config)

@bp.route("speech_settings", methods=["GET", "POST"])
def speech_settings():
    tskin_interface = get_interface(TSkinInterface)

    if not tskin_interface:
        flash("TSkin interface not available", "danger")
        return redirect(url_for("main.settings"))

    if request.method == "POST":
        scorer = get_from_request("scorer")

        if not scorer:
            flash("Please select a valid scorer", "danger")
            return redirect(url_for("tskin.speech_settings"))
        
        scorer_config = tskin_interface.get_scorer_config(scorer) if tskin_interface else None

        if not scorer_config:
            flash("Scorer not found in config", "danger")
            return redirect(url_for("tskin.speech_settings"))

        tskin_interface.set_scorer_config(scorer_config)
        flash("Speech saved!", "success")

    return render_template(
        "tskin/speech_settings.jinja", 
        scorers=tskin_interface.get_scores(), 
        selected_scorer=tskin_interface.get_selected_scorer()
    )

@bp.route("scan", methods=["GET"])
def scan():
    tskin_interface = get_interface(TSkinInterface)
    tskin_list = []

    if tskin_interface:
        tskin_list = tskin_interface.find_tskin_list()

    return json.dumps(tskin_list)

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
    tskin_interface = get_interface(TSkinInterface)

    if not tskin_interface:
        flash("TSkin interface not available", "danger")
        return redirect(url_for("main.main"))

    tskin_name = get_from_request("tskin_name")
    tskin_mac = get_from_request("tskin_mac")

    if not tskin_name or not tskin_mac:
        return redirect(url_for("tskin.add"))

    hand = get_from_request("hand")

    if not hand:
        flash("Please select a hand", "danger")
        return redirect(url_for("tskin.add"))

    models = tskin_interface.get_models(Hand(hand))

    if len(models) == 1:
        return redirect(url_for("tskin.save", tskin_name=tskin_name, tskin_mac=tskin_mac, hand=hand, model=models[0].name), code=307)

    return render_template("tskin/model.jinja", tskin_name=tskin_name, tskin_mac=tskin_mac, hand=hand, models=models)

@bp.route("/remove", methods=["GET"])
def remove():
    tskin_interface = get_interface(TSkinInterface)

    if not tskin_interface:
        flash("TSkin interface not available", "danger")
        return redirect(url_for("main.main"))

    tskin_interface.reset_tskin_config()

    return redirect(url_for("main.index"))

@bp.route("save", methods=["POST"])
def save():
    tskin_interface = get_interface(TSkinInterface)

    if not tskin_interface:
        flash("TSkin interface not available", "danger")
        return redirect(url_for("main.main"))

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

    tskin_interface.stop()
    tskin_interface.set_tskin_config(tskin_mac, hand, tskin_name, model_name)

    if not tskin_interface.get_selected_scorer():
        tskin_interface.set_scorer_config(DEFAULT_SCORER_CONFIG)

    if not tskin_interface.get_socket_config():
        tskin_interface.set_socket_config(DEFAULT_SOCKET_CONFIG)

    tskin_config = tskin_interface.get_tskin_config()
    if tskin_config is None:
        flash("TSkin configuration not found", "danger")
        return {"error": "TSkin configuration not found"}, 500

    tskin_interface.start()

    return json.dumps(tskin_config.toJSON()), 200

@bp.route("play", methods=["POST"])
def play(base_path = ""):
    # tskin = get_tskin()

    # if not tskin:
    #     return "Error: TSkin not present"
    
    # data = request.get_json(force=True)

    # if not data:
    #     return "Error: No JSON in request"
    
    # audio_file = data.get("audio_file")

    # if not audio_file:
    #     return "Error: No audio_file in JSON"
    
    # base_path = data.get("base_path")

    # if base_path:
    #     audio_file = path.join(base_path, audio_file)

    # tskin.play(audio_file)
    # TODO: implement play

    return "Ok"