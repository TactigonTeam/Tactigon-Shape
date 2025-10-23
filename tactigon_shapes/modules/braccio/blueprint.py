import sys
import json
import asyncio
from bleak import BleakScanner
from flask import Blueprint, render_template, redirect, url_for, flash

from tactigon_shapes.modules.braccio.extension import BraccioConfig, BraccioInterface
from tactigon_shapes.modules.braccio.manager import get_braccio_interface

from tactigon_shapes.config import check_config
from tactigon_shapes.utils.extensions import stop_apps
from tactigon_shapes.utils.request_utils import get_from_request

bp = Blueprint("braccio", __name__, url_prefix="/braccio", template_folder="templates")

@bp.before_request
def manage():
    stop_apps(BraccioInterface.__name__)

@bp.route("/")
@check_config
def index():
    app = get_braccio_interface()

    if not app:
        flash("Braccio interface not running", category="danger")
        return redirect(url_for("main.index"))

    return render_template("braccio/index.jinja", configured=app.configured, config=app.config)

if sys.platform == "darwin":
    @bp.route("scan")
    @check_config
    def scan():
        app = get_braccio_interface()

        if not app:
            flash("Braccio interface not running", category="danger")
            return redirect(url_for("main.index"))
        
        async def find_devices():
            devices = await BleakScanner.discover(cb=dict(use_bdaddr=True))
            return filter(lambda d: str(d.name).startswith("ADA"), devices)

        devices = [{"name": d.name, "id": d.address, "address": str(d.details[0].identifier())} for d in asyncio.run(find_devices())]

        return json.dumps(devices)
else:
    @bp.route("/scan")
    @check_config
    def scan():
        app = get_braccio_interface()

        if not app:
            flash("Braccio interface not running", category="danger")
            return redirect(url_for("main.index"))
        
        async def find_devices():
            devices = await BleakScanner.discover()
            return filter(lambda d: str(d.name).upper().startswith("ADA") , devices)

        devices = [{"name": d.name, "id": d.address, "address": d.address} for d in asyncio.run(find_devices())]

        return json.dumps(devices)

@bp.route("/save", methods=["POST"])
@check_config
def save():
    app = get_braccio_interface()

    if not app:
        flash("Braccio interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    name = get_from_request("name")
    address = get_from_request("address")

    if name is None:
        flash("Cannot save Braccio configurations. Name missing", category="danger")
        return redirect(url_for("braccio.index"))
    
    if address is None:
        flash("Cannot save Braccio configurations. Address missing", category="danger")
        return redirect(url_for("braccio.index"))
    
    new_config = BraccioConfig(name, address)
    app.save_config(new_config)

    flash("Braccio configured succesfully", category="success")
    return redirect(url_for("braccio.index"))

@bp.route("/remove")
@check_config
def remove():
    app = get_braccio_interface()

    if not app:
        flash("Braccio interface not running", category="danger")
        return redirect(url_for("main.index"))

    app.reset_config()

    flash("Braccio configuration removed!", category="success")
    return redirect(url_for("braccio.index"))

@bp.route("/start")
@check_config
def start():
    app = get_braccio_interface()

    if not app:
        flash("Braccio interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    if app.config is None:
        flash("Invalid configuration", category="danger")
        return redirect(url_for("braccio.index"))
    
    app.start()
    flash("Braccio started!", category="success")
    return redirect(url_for("braccio.index"))

@bp.route("/stop")
def stop():
    app = get_braccio_interface()

    if not app:
        flash("Braccio interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    app.stop()
    flash("Braccio stopped", category="success")
    return redirect(url_for("braccio.index"))