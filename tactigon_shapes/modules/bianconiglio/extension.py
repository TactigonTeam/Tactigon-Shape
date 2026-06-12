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
    config: BianconiglioConfig | None

    def __init__(self, config_file_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(BianconiglioInterface.__name__)

        self._dataframe = pd.DataFrame()
        self.config = None

        self.config_file_path = config_file_path
        self.load_config()

        #self.models: list = [("modello1","model1"),("modello2","model2")]

        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[BianconiglioInterface.__name__] = self
    
    def get_shape_blocks(self):
        updated_models = self.get_models()
        if updated_models and len(updated_models) > 0 and updated_models[0].get("model_id") != "---":
             self.models = updated_models
        return {
            "states": [(state.name, state.value) for state in BianconiglioState],
            "models": [(m["description"], m["model_id"]) for m in self.models]
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

                self._logger.info("Bianconiglio configuration loaded. URL: %s, Base: %s", self.config.url, self.config.base_endpoint)

                self.models = self.get_models()

        else:
            self.config = None
            self._logger.warning("Bianconiglio configuration file not found at %s, will use defaults", self.config_file)
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

            data =res.json()
            clean_res = [f"ID: {m.get('model_id')} - Desc: {m.get('description')}" for m in data['models_infos']]
            self._logger.info(f"GET %s response: %s", url, res.status_code)
            self._logger.info("\n".join(clean_res))
            return res.json()

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None

    def get_models(self) -> list:
        self._logger.info("entro in get models - 0")
        """Populate the models list with a get request"""
        if not self.config:
            self._logger.info("config non trovata - 1")
            return []
        try:
            url = f"{self.config.url}{self.config.base_endpoint}"
            self._logger.info("url =" + url)
            res = self.do_get(url)

            if not res:
                self._logger.info("nessun modello trovato")
                return [{
                    "model_id": "---",
                    "description": "no models available"
                    }]
            
            return res["models_infos"]

        except TimeoutError:
            logging.error(f" timeout api get models")

        except Exception as e:
            logging.error(f" errore get models: {e}")

        

        return []
    
    def get_status(self, model_id: str) -> str | None:
        """Function to get"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return None
        url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.status_endpoint}"
        
        try:
            response = self.do_get(url, timeout=5)
            if response:
                return response.get("state", "ERROR")            
            return None
        
        except Exception as e:
            self._logger.error(f"Error getting status: {e}")
            return None
        
    def get_log(self, model_id: str):
        """Function to get the logs about a specific model"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return None
        url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.log_endpoint}"

        try:
            response = self.do_get(url, timeout=5)       
            
            return response
        
        except Exception as e:
            self._logger.error(f"Error getting logs: {e}")
            return "ERROR"

    def train(self, description: str, data: pd.DataFrame, features: list[str], targets: list[str], url: str = None) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        url = url or f"{self.config.url}{self.config.base_endpoint}{self.config.train_endpoint}"

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
            if response and response.status_code == 200:
                self._logger.info("Train successful")
                self.models = self.get_models()
                return response.json()
            elif response:
                self._logger.error(f"Error from server during training. Code: {response.status_code}")
                return response.json()
            else:
                return {}
        except requests.exceptions.Timeout:
            self._logger.error("TIMEOUT: The training is taking too long.")
            return {}
        except requests.exceptions.RequestException as e:
            self._logger.error(f"Error connecting during training: {e}")
            return {}
        
    def retrain(self, model_desc: str, new_description: str, data: pd.DataFrame, features: list[str], targets: list[str]):
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        url = f"{self.config.url}{self.config.base_endpoint}/{model_desc}{self.config.retrain_endpoint}"

        response = self.train(new_description, data, features, targets, url)
        return response
        
    def predict(self, model_id: str, data: pd.DataFrame) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.predict_endpoint}"

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