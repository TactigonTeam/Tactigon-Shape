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

from flask import request, flash
from typing import Any

from tactigon_shapes.modules.shapes.manager import get_shapes_app


def get_from_request(name: str) -> str | None:
    if name in request.args:
        return request.args.get(name)
    
    if name in request.form:
        return request.form.get(name)
    
    return None

def check_empty_inputs(local_variables: Any) -> bool:
    for var_name, var_value in local_variables:
        if not var_name.startswith('_') and (var_value is None or var_value == ""):
            flash(f"{var_name.replace('_', ' ')} is required!", category="danger")
            return True

    return False

# decoratore per funzioni singole
def stop_shape(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        _shapes = get_shapes_app()
        if _shapes and _shapes.is_running:
            _shapes.stop()
            flash("Running Shape has been stopped to access the configuration.", category="warning")
        return func(*args, **kwargs)
    return wrapper

# chiamata per tutto un blueprint
def stop_running_shape():
    _shapes = get_shapes_app()
    if _shapes and (_shapes.is_running or _shapes.current_id):
        _shapes.stop()
        flash("Running Shape has been stopped to access the configuration.", category="warning")