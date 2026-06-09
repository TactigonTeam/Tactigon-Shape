import logging
import os
import json
import pandas as pd
import requests
from flask import Flask
from tactigon_shapes.modules.bianconiglio.models import BianconiglioState, BianconiglioConfig, DataFrameFileExtension
from tactigon_shapes.modules.file_manager.extension import FileManager

class BianconiglioInterface:
    config_file_path: str
    _dataframe: pd.DataFrame

    def __init__(self, config_file_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(BianconiglioInterface.__name__)
        self._dataframe = pd.DataFrame()
        self.config_file_path = config_file_path
        self.load_config()
        self.models: list = []

        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[BianconiglioInterface.__name__] = self
    
    def get_shape_blocks(self):

        return {
            "states": [(state.name, state.value) for state in BianconiglioState],
            "models": [model for model in self.models]
        }
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    @property
    def dataframe_extensions(self) -> list[str]:
        return [f".{e.value}" for e in DataFrameFileExtension]
    
    def load_config(self):
        """loads configuration from Bianconiglioconfig JSON
        """
        self._logger.info(f"Bianconiglio configuration path: {self.config_file_path}")
        self._logger.info(f"Bianconiglio configuration file: {self.config_file}")
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)
                self.config = BianconiglioConfig.FromJSON(config_data)
                self._logger.info("Bianconiglio configuration loaded. %s", self.config)
                self.models = self.get_models()

        else:
            self.config = None
            self._logger.warning("Bianconiglio configuration file not found at %s, will use defaults", self.config_file)
            self.config = BianconiglioConfig.Default()
            self.models = []
            self.save_config()

    def save_config(self):
        """Saves config onto a file"""
        if not os.path.exists(self.config_file_path):
            os.makedirs(self.config_file_path)

        with open(self.config_file, "w") as f:
            json.dump(self.config.toJSON(), f, indent=2)

        self._logger.info("Bianconiglio configuration saved.")

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

        try:
            res = requests.post(
                url,
                json=payload,
                timeout=timeout
                )
            
            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        except requests.exceptions.Timeout:
            self._logger.error(f"POST {url} Timeout expired (server hanging)")
        except Exception as e:
            self._logger.warning("POST %s failed: %s", url, e)
               
        return None

    def do_get(self, url: str, timeout: int = 5) -> requests.Response | None:
        """function used to make a GET request,
        error 401 is checked in case credentials fail or timeout
        Args:
            url (str): request URL
            timeout (int): timeout in seconds
        Returns:
            dict | None: JSON of the response
        """
        if not self.config or not self.config.is_valid():
            return None

        try:
            res = requests.get(url, timeout)


            self._logger.debug("GET %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None

    def get_models(self) -> list:
        """Populate the models list with a get request"""
        models_list = []
        models_dict = {}
        try: 
            res = self.do_get(self.config.base_endpoint)
      
            if isinstance(res, dict) and res:
                """l'endpoint restituisce: dict {models: {model_id: model_state_string}}
                    oppure: dict {"models": "no models available"}"""
                models_dict = res.get("models")


            if isinstance(models_dict, dict) and res:
                models_list = list(models_dict.keys())

    
        except TimeoutError:
            logging.error(f" timeout api get models")

        except Exception as e:
            logging.error(f" errore get models: {e}")

        

        return models_list
    
    def get_status(self, model_id: str) -> str:
        """Function to get"""
        url = f"{self.config.base_endpoint}/{model_id}{self.config.status_endpoint}"
        
        try:
            response = self.do_get(url, timeout=5)
            if response and response.status_code == 200:
                data = response.json()
                return data.get("state", "ERROR")            
            return
        except Exception as e:
            self._logger.error(f"Error getting status: {e}")
            return "ERROR"
        
    def get_log(self, model_id: str):
        """Function to get the logs about a specific model"""
        url = f"{self.config.base_endpoint}/{model_id}{self.config.log_endpoint}"

        try:
            response = self.do_get(url, timeout=5)
            if response and response.status_code == 200:
                return response.json()          
            
            return response
        
        except Exception as e:
            self._logger.error(f"Error getting logs: {e}")
            return "ERROR"

    def train(self, description: str, data: pd.DataFrame, features: list[str], targets: list[str], url: str | None) -> dict:
        if not url:
            url = f"{self.config.url}{self.config.train_endpoint}"

        self._logger.info(f"Training data type: {type(data)}")
        self._logger.info(f"Training datas:\ndata: {data}\nfeatures: {features}\ntargets: {targets}")
        
        payload = {
            "description": description,
            "data": data.to_dict(orient="records"),
            "features": features,
            "targets": targets
        }

        self._logger.info(f"Payload:\n{payload}")

        try:
            # 5 seconds timeout because the response will arrive immediatley while the training process will keep going in background
            response = self.do_post(url, payload, timeout=5)
            if response.status_code == 200:
                self._logger.info("Train successful")
                return response.json()
            else:
                self._logger.error(f"Error from server during training. Code: {response.status_code}")
                return response.json()
        except requests.exceptions.Timeout:
            self._logger.error("TIMEOUT: The training is taking too long.")
            return {}
        except requests.exceptions.RequestException as e:
            self._logger.error(f"Error connecting during training: {e}")
            return {}
        
    def retrain(self, model_id: str, description: str, data: pd.DataFrame, features: list[str], targets: list[str]):
        url = f"{self.config.url}/{model_id}{self.config.retrain_endpoint}"

        response = self.train(description, data, features, targets, url)
        return response
        
    def predict(self, model_id: str, data: pd.DataFrame) -> dict:

        url = f"{self.config.url}/{model_id}{self.config.predict_endpoint}"

        self._logger.info(f"Prediction datas:\ndata: {data}")
        self._logger.info(f"Data type: {type(data)}")

        payload = {
            "model_id": model_id,
            "data": data.to_dict(orient="records"),
        }

        self._logger.info(f"Payload:\n{payload}")

        try:
            response = self.do_post(url, payload, timeout=10)

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
            self._logger.error("Cannot read file into dataframe. %s", e.with_traceback)
            
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