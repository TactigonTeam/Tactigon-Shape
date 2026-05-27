import logging
import os
import json
import pandas as pd
import requests
from flask import Flask
from tactigon_shapes.modules.bianconiglio.models import BianconiglioState, BianconiglioConfig

class BianconiglioInterface:
    config_file_path: str
    #config: BianconiglioConfig | None

    def __init__(self, config_file_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(BianconiglioInterface.__name__)

        self.config_file_path = config_file_path
        self.load_config()

        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[BianconiglioInterface.__name__] = self
    
    def get_shape_blocks(self):

        return {
            "states": [(state.name, state.value) for state in BianconiglioState]
        }
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    def load_config(self):
        """loads configuration from Bianconiglioconfig JSON
        """
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)
                self.config = BianconiglioConfig.FromJSON(config_data)
                self._logger.info("Bianconiglio configuration loaded. %s", self.config)
        else:
            self.config = None
            self._logger.warning("Bianconiglio configuration file not found at %s, will use defaults", self.config_file)
            self.config = BianconiglioConfig.Default()
            self.save_config()

    def save_config(self):
        """Saves config onto a file"""
        if not os.path.exists(self.config_file_path):
            os.makedirs(self.config_file_path)

        with open(self.config_file, "w") as f:
            json.dump(self.config.toJSON(), f, indent=2)

        self._logger.info("Bianconiglio configuration saved.")

    def do_post(self, url: str, payload: dict) -> requests.Response | None:
        """function used to make a POST request.
        header contains authentication token.
        error 401 is checked in case credentials fail or timeout
        Args:
            url (str): request URL
            payload (object): the message we want to post

        Returns:
            requests.Response | None:request response
        """
        if not self.config or not self.config.is_valid():
            return None

        try:
            res = requests.post(
                url,
                json=payload,
                timeout=10
                )
            
            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        except requests.exceptions.Timeout:
            self._logger.error(f"POST {url} Timeout expired (server hanging)")
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
        if not self.config or not self.config.is_valid():
            return None

        try:
            res = requests.get(url, timeout=5)


            self._logger.debug("GET %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None

    def status(self) -> str:
        url = f"{self.config.url}{self.config.status_endpoint}"
        
        response = self.do_get(url)
        if response and response.status_code == 200:
            data = response.json()
            return data.get("state", "ERROR")
            
        return "ERROR"

    def train(self, data: list[dict], features: list[str], targets: list[str]) -> dict:
        url = f"{self.config.url}{self.config.train_endpoint}"

        self._logger.info(f"Training datas:\ndata: {data}\nfeatures: {features}\ntargets: {targets}")
        
        payload = {
            "data": data,
            "features": features,
            "targets": targets
        }

        self._logger.info(f"Payload:\n{payload}")

        response = self.do_post(url, payload)

        if response and response.status_code == 200:
            self._logger.info("Training successful")
            return response.json()
        else:
            self._logger.error(f"Training failed. Status: {response.status_code if response else 'None'}")
            if response:
                self._logger.error(f"Error details: {response.text}")
            return {}
    
    def predict(self, data: list) -> dict:

        url = f"{self.config.url}{self.config.predict_endpoint}"

        self._logger.info(f"Prediction datas:\ndata: {data}")
        self._logger.info(f"Data type: {type(data)}")

        # if isinstance(data, dict):
        #     data = [data]
        
        try:
            response = self.do_post(url, data)

            if response and response.status_code == 200:
                self._logger.info("Predict successful")
                return response.json()
            else:
                self._logger.error(f"Predict failed: {response.status_code if response else 'No Response'}")
                if response:
                    self._logger.error(f"Error details: {response.text}")
                return {}

        except Exception as e:
            self._logger.error(f"Predict error: {e}")
            return {}