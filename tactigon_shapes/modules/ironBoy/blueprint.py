import sys
import json
import asyncio
from bleak import BleakScanner
from flask import Blueprint, render_template, redirect, url_for, flash

from .extension import IronBoyConfig, IronBoyInterface
from .manager import get_ironBoy_interface

from ...config import check_config
from ...utils.extensions import stop_apps
from ...utils.request_utils import get_from_request

bp = Blueprint("ironBoy", __name__, url_prefix="/ironBoy", template_folder="templates")

@bp.before_request
def manage():
    stop_apps(IronBoyInterface.__name__)

@bp.route("/")
@check_config
def index():
    app = get_ironBoy_interface()

    if not app:
        flash("IronBoy interface not running", category="danger")
        return redirect(url_for("main.index"))

    return render_template("ironBoy/index.jinja", configured=app.configured, config=app.config)

if sys.platform == "darwin":
    @bp.route("scan")
    @check_config
    def scan():
        app = get_ironBoy_interface()

        if not app:
            flash("ironboy interface not running", category="danger")
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
        app = get_ironBoy_interface()

        if not app:
            flash("ironBoy interface not running", category="danger")
            return redirect(url_for("main.index"))
        
        async def find_devices():
            devices = await BleakScanner.discover()
            return filter(lambda d: str(d.name).startswith("A") , devices)

        devices = [{"name": d.name, "id": d.address, "address": d.address} for d in asyncio.run(find_devices())]

        return json.dumps(devices)

@bp.route("/save", methods=["POST"])
@check_config
def save():
    app = get_ironBoy_interface()

    if not app:
        flash("ironboy interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    name = get_from_request("name")
    address = get_from_request("address")

    if name is None:
        flash("Cannot save ironboy configurations. Name missing", category="danger")
        return redirect(url_for("ironBoy.index"))
    
    if address is None:
        flash("Cannot save ironboy configurations. Address missing", category="danger")
        return redirect(url_for("ironBoy.index"))
    
    new_config = IronBoyConfig(name, address)
    app.save_config(new_config)

    flash("ironboy configured succesfully", category="success")
    return redirect(url_for("ironBoy.index"))

@bp.route("/remove")
@check_config
def remove():
    app = get_ironBoy_interface()

    if not app:
        flash("ironboy interface not running", category="danger")
        return redirect(url_for("main.index"))

    app.reset_config()

    flash("ironBoy configuration removed!", category="success")
    return redirect(url_for("ironBoy.index"))

@bp.route("/start")
@check_config
def start():
    app = get_ironBoy_interface()

    if not app:
        flash("ironBoy interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    if app.config is None:
        flash("Invalid configuration", category="danger")
        return redirect(url_for("ironBoy.index"))
    
    app.start()
    flash("ironBoy started!", category="success")
    return redirect(url_for("ironBoy.index"))

@bp.route("/stop")
def stop():
    app = get_ironBoy_interface()

    if not app:
        flash("ironBoy interface not running", category="danger")
        return redirect(url_for("main.index"))
    
    app.stop()
    flash("ironBoy stopped", category="success")
    return redirect(url_for("ironBoy.index"))