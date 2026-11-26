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


from flask import Blueprint, render_template, redirect, url_for

from tactigon_shapes import __version__


from tactigon_shapes.utils.request_utils import stop_shape

bp = Blueprint('main', __name__, template_folder="main")

@bp.route("/", methods=["GET"])

def index():
    return redirect(url_for("shapes.index"))

@bp.route("/settings")
@stop_shape
def settings():
    return render_template("info.jinja", version=__version__)