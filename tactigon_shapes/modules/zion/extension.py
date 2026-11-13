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

import logging
import os
import json
import requests

from flask import Flask

from tactigon_shapes.interfaces.interface import Interface
from tactigon_shapes.modules.zion.models import AlarmStatus, ZionConfig, Device, Scope, AlarmSearchStatus, AlarmSeverity, DeviceAlarm

APPLICATION_JSON = 'application/json'

class ZionInterface(Interface):
    _config: ZionConfig | None = None

    config_class = ZionConfig
    devices: list[Device] = []

    @property
    def can_run(self) -> bool:
        return bool(self._config and self._config.is_valid())
    
    @property
    def running(self) -> bool:
        return bool(self._config and self._config.is_valid())

    @property
    def token(self) -> str:
        return self._config.token if self._config else ""
    
    @token.setter
    def token(self, token: str):
        if self._config:
            self._config.token = token

    def post_load_config(self):
        if self._config:
            self.devices = self.get_devices()
        else:
            self.devices = []
        
    def get_shape_blocks(self):
        """the lists that create the basic blocks in Shape used in the Zion Api.
        these are shown as fields dropdowns and are included in the other blocks.

        Returns:
            _type_: dict of lists("devices","scopes","alarmSeverity","alarmSearchStatus")
        """
        return {
            "devices": [(d.name, d.id.id) for d in self.devices],
            "scopes": [(s.name, s.value) for s in Scope],
            "alarmSeverity": [(s.name, s.value) for s in AlarmSeverity],
            "alarmSearchStatus": [(s.name, s.value) for s in AlarmSearchStatus],
        }
    
    def do_post(self, url: str, payload: object) -> requests.Response | None:
        """function used to make a POST request.
        header contains authentication token.
        error 401 is checked in case credentials fail or timeout
        Args:
            url (str): request URL
            payload (object): the message we want to post

        Returns:
            requests.Response | None:request response
        """
        if not self._config or not self._config.is_valid():
            return None

        headers = {
            "accept": APPLICATION_JSON,
            "X-Authorization": f"Bearer {self.token}"
        }
        try:
            res = requests.post(
                url,
                json=payload,
                headers=headers
                )
            
            if res.status_code == 401:
                token = self.refresh_token(self._config.url, self._config.username, self._config.password)
                
                if not token:
                    return None
                
                self.token = token            
                return self.do_post(url, payload)

            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        
        except Exception as e:
            self._logger.warning("POST %s failed: %s", url, e)
               
        return None

    def do_get(self, url: str) -> requests.Response | None:
        """function used to make a GET request,
        error 401 is checked in case credentials fail or timeout
        Args:
            url (str): request URL
        Returns:
            dict | None: JSON of the response
        """
        if not self._config or not self._config.is_valid():
            return None

        headers = {
            "accept": APPLICATION_JSON,
            "X-Authorization": f"Bearer {self.token}"
        }

        try:
            res = requests.get(url, headers=headers, timeout=5)

            if res.status_code == 401:
                token = self.refresh_token(self._config.url, self._config.username, self._config.password)
                
                if not token:
                    return None
                
                self.token = token            
                return self.do_get(url)

            self._logger.debug("GET %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None
    
    def do_delete(self, url: str) -> requests.Response | None:
        """function used to make a DELETE request,
        error 401 is checked in case credentials fail or timeout
        Args:
            url (str): request URL
        Returns:
            dict | None: response status code
        """

        if not self._config or not self._config.is_valid():
            return None

        headers = {
        "accept": APPLICATION_JSON,
        "X-Authorization": f"Bearer {self.token}"
        }

        try:
            res = requests.delete(url, headers=headers, timeout=5)

            if res.status_code == 401:
                token = self.refresh_token(self._config.url, self._config.username, self._config.password)
                
                if not token:
                    return None
                
                self.token = token
                return self.do_delete(url)        

            self._logger.debug("DELETE %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("DELETE %s failed: %s", url, e)
            
        return None

    def refresh_token(self, url, username: str, password: str) -> str | None:
        """make login to refresh auth token

        Args:
            url (_type_): config URL for login
            username (str): config username
            password (str): config password

        Returns:
            str | None: auth token 
        """
        headers = {
            "Content-Type": APPLICATION_JSON,
            "accept": APPLICATION_JSON,
        }
        try:
            res = requests.post(
                f"{url}api/auth/login",
                headers=headers,
                json={"username": username, "password": password}
            )

            res.raise_for_status()
            
            data = res.json()
            self._logger.info("Token refreshed successfully.")
            return data["token"]

        except Exception as e:
            self._logger.warning("Refresh token error: %s", e)

        return None

    def get_devices(self, size: int = 20, page: int = 0) -> list[Device]:
        """Populate device list trough connection with Zion Devices

        Returns:
            bool: list is populated 
        """

        device_list = []

        if not self._config or not self._config.is_valid():
            return device_list
        
        hasNext = True
        
        while hasNext:
            url = f"{self._config.url}api/tenant/devices?pageSize={size}&page={page}"
            res = self.do_get(url)

            if not res:
                break

            devices = res.json()

            for device in devices["data"]:
                device_list.append(
                    Device.FromZION(device)
                ) 
        
            hasNext = devices.get("hasNext", False)
            page += 1
        
        self._logger.info("Retrieved %d devices from Zion.", len(device_list))
        return device_list
    
    def device_last_telemetry(self, device_id: str, keys: str = "") -> dict | None:
        """returns the chosen device's last telemetry in json format,
        called by zion_device_last_telemetry

        Args:
            device_id (str): id of device
            keys (str, optional): keys to filter result. Defaults to "".

        Returns:
            dict | None: dictionary
        """
        if not self._config or not self._config.is_valid():
            return None
        
        url = f"{self._config.url}api/plugins/telemetry/DEVICE/{device_id}/values/timeseries?useStrictDataTypes=true"
        if keys:
            url += f"&keys={keys}"

        res = self.do_get(url)

        if not res or res.status_code != 200:
            self._logger.error("Failed to retrieve telemetry for device %s: %s.", device_id, res.status_code if res else "No response")
            return None
        
        telemetry = res.json()

        ret = {}
        for k in telemetry:
            ret[k] = telemetry[k][0]["value"]

        self._logger.info("Retrieved telemetry for device %s: %s", device_id, ret)
        return ret
    
    def device_attr(self, device_id: str, scope: Scope = Scope.SERVER, keys: str = "") -> dict | None:
        """returns all attributes from chosen device,
        called by zion_device_attribute

        Args:
            device_id (str): id of device
            scope (Scope, optional): specifies where the data is searched. Defaults to Scope.SERVER.
            keys (str, optional): can be used to filter. Defaults to "".

        Returns:
            dict | None: dict of device attributes
        """
        if not self._config or not self._config.is_valid():
            return None
        
        url = f"{self._config.url}api/plugins/telemetry/DEVICE/{device_id}/values/attributes/{scope.value}"
        if keys:
            url += f"?keys={keys}"

        res = self.do_get(url)

        if not res or res.status_code != 200:
            self._logger.error("Failed to retrieve attributes for device %s: %s.", device_id, res.status_code if res else "No response")
            return None
        
        attributes = res.json()

        ret = {}
        for attr in attributes:
            ret[attr["key"]] = attr["value"]

        self._logger.info("Retrieved attributes for device %s: %s", device_id, ret)
        return ret
    
    def device_alarm(self, device_id: str, severity: AlarmSeverity = AlarmSeverity.CRITICAL, search_status: AlarmSearchStatus = AlarmSearchStatus.ACTIVE, size: int = 20, page: int = 0) -> list[DeviceAlarm]:
        """Returns a page of alarms for the selected device.
        called by zion_device_alarm
        Args:
            device_id (str): id of device
            severity (AlarmSeverity, optional): severity of alarm. Defaults to AlarmSeverity.CRITICAL.
            search_status (AlarmSearchStatus, optional):one of the AlarmStatus enumeration value. Defaults to AlarmSearchStatus.ACTIVE.
            size (int, optional): Maximum amount of entities in a one page. Defaults to 20.
            page (int, optional): Sequence number of page starting from 0. Defaults to 0.

        Returns:
            list[dict] | None: list of dicts
        """

        alarms: list[DeviceAlarm] = []
        
        if not self._config or not self._config.is_valid():
            return alarms

        hasNext = True
        
        while hasNext:
            url = f"{self._config.url}api/alarm/DEVICE/{device_id}?searchStatus={search_status.value}&textSearch={severity.value}&pageSize={size}&page={page}"
            res = self.do_get(url)

            if not res:
                break

            alarms_response = res.json()

            for alarm in alarms_response["data"]:
                alarms.append(DeviceAlarm.FromZION(alarm))

            hasNext = alarms_response.get("hasNext", False)
            page += 1

        self._logger.info("Retrieved %d alarms for device %s.", len(alarms), device_id)
        return alarms

    def send_device_last_telemetry(self, device_id: str, payload: dict) -> bool:
        """Creates or updates the device time-series data based on the device Id and request payload.
        Args:
            device_id (str): id of device
            payload (dict): dict of keys and value we want to send

        Returns:
            bool: response code is 200
        """
        if not self._config or not self._config.is_valid():
            return False
        
        url = f"{self._config.url}api/plugins/telemetry/DEVICE/{device_id}/timeseries/ANY?scope=ANY"

        res = self.do_post(url, payload)
        
        if not res or res.status_code != 200:
            self._logger.error("Failed to send telemetry to device %s: %s.", device_id, res.status_code if res else "No response")
            return False
        
        self._logger.info("Telemetry sent to device %s: %s.", device_id, payload)
        
        return True
    
    def send_device_attr(self, device_id: str, payload: dict, scope: Scope = Scope.SERVER) -> bool:
        if not self._config or not self._config.is_valid():
            return False
        
        url = f"{self._config.url}api/plugins/telemetry/{device_id}/{scope.value}"

        res = self.do_post(url, payload)

        if not res or res.status_code != 200:
            self._logger.error("Failed to send attributes to device %s: %s.", device_id, res.status_code if res else "No response")
            return False
        
        self._logger.info("Attributes sent to device %s: %s.", device_id, payload)
        return True
    
    def upsert_device_alarm(self, device_id: str, alarm_name: str, alarm_type: str, severity: AlarmSeverity = AlarmSeverity.CRITICAL, status: AlarmStatus = AlarmStatus.CLEARED_UNACK) -> bool:
        """insert or update device alarm,
         called by zion_send_device_alarm
        Args:
            device_id (str): id of device
            alarm_name (str): alarm's name
            alarm_type (str): alarm's type
            severity (AlarmSeverity, optional): severity of alarm. Defaults to AlarmSeverity.CRITICAL.
            status (AlarmStatus, optional): alarm's status. Defaults to AlarmStatus.CLEARED_UNACK.

        Returns:
            bool: connection was successful
        """
        if not self._config or not self._config.is_valid():
            return False
        
        url = f"{self._config.url}api/alarm"

        device = next(filter(lambda d: d.id.id == device_id, self.devices), None)

        if not device:
            return False
        
        payload = device.to_alarm()
        payload["name"] = alarm_name
        payload["type"] = alarm_type
        payload["severity"] = severity.value
        payload["status"] = status.value

        res = self.do_post(url, payload)

        if not res or res.status_code != 200:
            self._logger.error("Failed to insert alarm for device %s: %s.", device_id, res.status_code if res else "No response")
            return False

        self._logger.info("Alarm inserted for device %s: %s.", device_id, payload)
        return True

    def delete_device_attr(self, device_id: str, scope: Scope = Scope.SERVER, key : str="") -> bool:
        """delete device's attribute, 
        called by zion_delete_device_attr

        Args:
            device_id (str): device's id 
            scope (Scope, optional): where the data needs to be searched. Defaults to Scope.SERVER.
            key (str): specifies the key we want to delete. Defaults to "".

        Returns:
            bool: operation result
        """      
        if not self._config or not self._config.is_valid():
            return False
        
        if not self.token:
            token = self.refresh_token(self._config.url, self._config.username, self._config.password)
            if not token:
                return False
            self.token = token
        
        url = f"{self._config.url}api/plugins/telemetry/DEVICE/{device_id}/{scope.value}?keys={key}"

        res = self.do_delete(url)

        if not res or res.status_code != 200:
            self._logger.error("Failed to delete attribute %s for device %s: %s.", key, device_id, res.status_code if res else "No response")
            return False
        
        self._logger.info("Attribute %s deleted for device %s.", key, device_id)

        return True