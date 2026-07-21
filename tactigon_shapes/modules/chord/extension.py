import logging
import os
import json
import pandas as pd
import requests
from flask import Flask
from tactigon_shapes.modules.chord.models import ChordState, ChordConfig, DataFrameFileExtension, ModelInfo
from tactigon_shapes.modules.file_manager.extension import FileManager

class ChordInterface:
    config_file_path: str
    _dataframe: pd.DataFrame
    config: ChordConfig | None

    def __init__(self, config_file_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(ChordInterface.__name__)

        self._dataframe = pd.DataFrame()
        self.config = None

        self.config_file_path = config_file_path
        self.load_config()

        #self.models: list = [("modello1","model1"),("modello2","model2")]

        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[ChordInterface.__name__] = self
        self.update_models()
    
    def get_shape_blocks(self):
        self.update_models()
        return {
            "states": [(state.name, state.value) for state in ChordState],
            "models": [(m["description"], m["model_id"]) for m in self.models] if self.models else [["No models found", ""]]
        }
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    @property
    def dataframe_extensions(self) -> list[str]:
        return [f".{e.value}" for e in DataFrameFileExtension]
    
    def load_config(self):
        self._logger.info(f"Chord configuration path: {self.config_file_path}")
        self._logger.info(f"Chord configuration file: {self.config_file}")
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)
                self.config = ChordConfig.FromJSON(config_data)

                self._logger.info("Chord configuration loaded. URL: %s", self.config.url)

                self.models = self.get_models_info()

                self._logger.info(f"models: {self.models}")
        else:
            self.config = None
            self._logger.warning("Chord configuration file not found at %s, will use defaults", self.config_file)
            self.models = []

    def save_config(self):
        """Saves config onto a file"""
        if not self.config:
            self._logger.warning("Config is not loaded, cannot save.")
            return
            
        if not os.path.exists(self.config_file_path):
            os.makedirs(self.config_file_path)

        with open(self.config_file, "w") as f:
            json.dump(self.config.toJSON(), f, indent=2)

        self._logger.info("Chord configuration saved.")

    def do_post(self, url: str, payload: dict, timeout: int = 10) -> requests.Response | None:
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
        
        self._logger.info(f"POST: {url}, payload: {payload}")

        try:
            res = requests.post(
                url,
                json=payload,
                timeout=timeout
            )

            res.raise_for_status()
            
            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        except requests.exceptions.Timeout:
            self._logger.error(f"POST {url} Timeout expired (server hanging)")
        except Exception as e:
            self._logger.error("POST %s failed: %s", url, e)
               
        return None

    def do_get(self, url: str, timeout: int = 5) -> dict | None:
        """function used to make a GET request,
        error 401 is checked in case credentials fail or timeout
        Args:
            url (str): request URL
            timeout (int): timeout in seconds
        Returns:
            dict | None: JSON of the response
        """
        if not self.config or not self.config.is_valid():
            self._logger.info("sto per tornare none")
            return None

        try:
            res = requests.get(url)
            res.raise_for_status()

            self._logger.info(f"GET %s response: %s", url, res.status_code)

            return res.json()

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
        
        return None
    
    def update_models(self):
        if not self.config:
            self._logger.info("config non trovata - 1")
            self.models = []
            return
        
        res = self.do_get(self.config.models_endpoint())

        if not res or isinstance(res.get("models_infos"), str):
            self._logger.info("Nessun modello trovato o api vuota")
            self.models = []
            return
        
        self.models = res.get("models_infos", [])

    def get_models_info(self) -> list:
        self.update_models()
        return self.models
    
    def get_status(self, model_id: str) -> str:
        """Function to get"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return "Error loading Chord config"
        
        response = self.do_get(self.config.status_endpoint(model_id))

        if response:
            return response.get("state", "Error getting status")
        
        return "Error getting status"
        
    def get_log(self, model_id: str):
        """Function to get the logs about a specific model"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return None

        return self.do_get(self.config.log_endpoint(model_id))
        
    def train(self, description: str, data: pd.DataFrame, features: list[str], targets: list[str], url: str | None = None) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        
        payload = {
            "description": description,
            "data": data.to_dict(orient="records"),
            "features": features,
            "targets": targets
        }

        return self._train(self.config.train_endpoint(), payload)
        
    def retrain(self, model_id: str, data: pd.DataFrame, features: list[str], targets: list[str]):
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        
        payload = {
            "data": data.to_dict(orient="records"),
            "features": features,
            "targets": targets
        }

        return self._train(self.config.retrain_endpoint(model_id), payload)
        
    def predict(self, model_id: str, data: pd.DataFrame) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}

        self._logger.info(f"Prediction datas:\ndata: {data}")
        self._logger.info(f"Data type: {type(data)}")

        payload = {
            "description": ",".join(data.columns),
            "model_id": model_id,
            "data": data.to_dict(orient="records"),
        }

        self._logger.info(f"Payload:\n{payload}")

        response = self.do_post(self.config.predict_endpoint(model_id), payload, timeout=10)

        return response.json() if response else {}

    def file_to_dataframe(self,file_path: str) -> pd.DataFrame | None:

        if FileManager.get_file_extension(file_path) not in self.dataframe_extensions:
            self._logger.error("File type not supported.")
            return None

        try:
            df = None

            if file_path.endswith('.csv'):
                df = pd.read_csv(file_path)

            elif file_path.endswith('.json'):
                df = pd.read_json(file_path)

            # elif file_path.endswith(('.txt', '.md')):
            #     with open(file_path, 'r', encoding='utf-8') as f:
            #         lines = [line.strip() for line in f.readlines() if line.strip()]

            #     return pandas.DataFrame(lines, columns=['content'])
            
            return df
        except Exception as e:
            self._logger.error(f"Cannot read file into dataframe: {str(e)}", exc_info=True)
            
        return None
    
    def get_dataframe(self, file_path: str) -> pd.DataFrame | None:
            
        new_df = self.file_to_dataframe(file_path)
        
        if new_df is None:
            self._logger.error(f"Cannot get dataframe from file {file_path}")
            return None
        
        if self._dataframe.empty:
            self._logger.warning(f"Context _dataframe is empty or missing, initializing with {file_path}")
            self._dataframe = new_df
        else:
            self._logger.warning(f"Dataframe already exists. Overriding with {file_path}.")
            self._dataframe = new_df
            # self._dataframe = pd.concat([self._dataframe, new_df], ignore_index=True)
       
        self._logger.info(f"Added {file_path} to context")
        return self._dataframe
    
    def _train(self, url: str, payload: dict) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        
        self._logger.info(f"Payload:\n{payload}")

        # 5 seconds timeout because the response will arrive immediatley while the training process will keep going in background
        response = self.do_post(url, payload, timeout=5)
        if response:
            self._logger.info("Train successful")
            self.models = self.get_models_info()
            return response.json()
        else:
            return {}