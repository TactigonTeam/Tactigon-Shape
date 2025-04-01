import os
import json
import requests

from flask import Flask
from typing import Optional, List

from .models import ZionConfig, Device, Scope, AlarmSearchStatus, AlarmSeverity

APPLICATION_JSON = 'application/json'

class ZionInterface:
    config_file_path: str
    config: Optional[ZionConfig]

    devices: List[Device] = []
    
    def __init__(self, config_file_path: str, app: Optional[Flask] = None):
        self.config_file_path = config_file_path
        self.load_config()

        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[ZionInterface.__name__] = self

    @property
    def configured(self) -> bool:
        return False if self.config is None else True
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    @property
    def token(self) -> Optional[str]:
        return self.config.token if self.config else None
    
    def load_config(self):
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                self.config = ZionConfig.FromJSON(json.load(f))
                self.get_devices()
        else:
            self.config = None
            self.devices = []

    def save_config(self, config: ZionConfig):
        if not os.path.exists(self.config_file_path):
            os.makedirs(self.config_file_path)

        with open(self.config_file, "w") as f:
            json.dump(config.toJSON(), f, indent=2)

        self.load_config()

    def reset_config(self):       
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            os.remove(self.config_file)

        self.load_config()
    
    def get_shape_blocks(self):
        return {
            "devices": [(d.name, d.id) for d in self.devices],
            "scopes": [(s.name, s.value) for s in Scope],
            "alarmSeverity": [(s.name, s.value) for s in AlarmSeverity],
            "alarmSearchStatus": [(s.name, s.value) for s in AlarmSearchStatus],
        }

    def do_post(self, url: str, payload: object):
        if not self.config:
            return None
        
        if not self.config.token:
            token = self.refresh_token(self.config.url, self.config.username, self.config.password)
            
            if not token:
                return None
            
            self.config.token = token

        headers = {
            "accept": APPLICATION_JSON,
            "X-Authorization": f"Bearer {self.token}"
        }

        res = requests.post(
            url,
            json=payload,
            headers=headers
            )
        
        if res.status_code == 401:
            token = self.refresh_token(self.config.url, self.config.username, self.config.password)
            
            if not token:
                return None
            
            self.config.token = token            
            return self.do_post(url, payload)

    def do_get(self, url: str) -> Optional[dict]:

        if not self.config.token:
            token = self.refresh_token(self.config.url, self.config.username, self.config.password)
            
            if not token:
                return None
            
            self.config.token = token
        
        headers = {
            "accept": APPLICATION_JSON,
            "X-Authorization": f"Bearer {self.config.token}"
        }

        res = requests.get(
            url,
            headers=headers
        )

        if res.status_code == 401:
            token = self.refresh_token(self.config.url, self.config.username, self.config.password)
            
            if not token:
                return None
            
            self.config.token = token            
            return self.do_get(url)

        return res.json()

    def refresh_token(self, url, username: str, password: str) -> Optional[str]:
        headers = {
            "Content-Type": APPLICATION_JSON,
            "accept": APPLICATION_JSON,
        }

        res = requests.post(
            f"{url}api/auth/login",
            headers=headers,
            json={"username": username, "password": password}
        )
        
        if res.status_code != 200:
            return None
        
        data = res.json()
        return data["token"]
    
    def get_devices(self, size: int = 20, page: int = 0):
        if not self.config:
            return False
        
        url = f"{self.config.url}api/tenant/devices?pageSize={size}&page={page}"

        res = self.do_get(url)

        if not res:
            return False

        for device in res["data"]:
            self.devices.append(
                Device(device["id"]["id"], device["name"])
            )

        if res["hasNext"]:
            return self.get_devices(size, page+1)
        
        return True

    def device_last_telemetry(self, device_id: str, keys: str = "") -> Optional[dict]:
        if not self.config:
            return None
        
        url = f"{self.config.url}api/plugins/telemetry/DEVICE/{device_id}/values/timeseries"
        if keys:
            url += f"?keys={keys}"

        res = self.do_get(url)

        if not res:
            return None

        ret = {}
        for k in res:
            ret[k] = res[k][0]["value"]

        return ret
    
    def device_attr(self, device_id: str, scope: Scope = Scope.SERVER, keys: str = "") -> Optional[dict]:
        if not self.config:
            return None
        
        url = f"{self.config.url}api/plugins/telemetry/DEVICE/{device_id}/values/attributes/{scope.value}"
        if keys:
            url += f"?keys={keys}"

        attributes = self.do_get(url)

        if not attributes:
            return None

        ret = {}
        for attr in attributes:
            ret[attr["key"]] = attr["value"]

        return ret
    
    def device_alarm(self, device_id: str, severity: AlarmSeverity = AlarmSeverity.CRITICAL, search_status: AlarmSearchStatus = AlarmSearchStatus.ACTIVE, size: int = 20, page: int = 0) -> Optional[List[dict]]:
        if not self.config:
            return None
        
        url = f"{self.config.url}api/alarm/DEVICE/{device_id}?searchStatus={search_status.value}&textSearch={severity.value}&pageSize={size}&page={page}"

        alarms = self.do_get(url)

        if not alarms:
            return None

        ret = []
        for alarm in alarms["data"]:
            ret.append({"id": alarm["id"]["id"], "severity": alarm["severity"], "status": alarm["status"], "startTs": alarm["startTs"]})

        if alarms["hasNext"]:
            others = self.device_alarm(device_id, severity, search_status, size, page+1)
            if others:
                ret.extend(others)

        return ret
    
    def send_device_attr(self, device_id: str, payload: dict, scope: Scope = Scope.SERVER) -> Optional[dict]:
        if not self.config:
            return None
        
        url = f"{self.config.url}api/plugins/telemetry/{device_id}/{scope}"

        res = self.do_post(url, payload)

        print(res)

        if not res:
            return None

        return res
    
    def send_device_last_telemetry(self, device_id: str, payload: dict) -> Optional[dict]:
        if not self.config:
            return None
        
        url = f"{self.config.url}api/plugins/telemetry/DEVICE/{device_id}/timeseries/ANY?scope=ANY"

        res = self.do_post(url, payload)

        if not res:
            return None

        return res