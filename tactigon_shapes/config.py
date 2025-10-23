import json
from functools import wraps
from os import path
from flask import redirect, url_for

from tactigon_shapes.models import BASE_PATH, AppConfig

config_file_path = path.join(BASE_PATH, "config")
config_file = path.join(config_file_path, "config.json")

if path.exists(config_file_path) and path.exists(config_file):
    with open(config_file, "r") as cf:
        app_config = AppConfig.FromJSON(json.load(cf), config_file)
else:
    app_config = AppConfig.Default(config_file)
    app_config.save()


def check_config(func):
    @wraps(func)
    def inner(*args, **kwargs):
        if app_config.TSKIN:
            return func(*args, **kwargs)
        
        return redirect(url_for("tskin.add"))

    return inner