import logging
import os
import json
import pandas as pd
import requests
import httpx
from flask import Flask
from contextlib import contextmanager
import os

from tactigon_shapes.modules.bianconiglio.models import (
    XgbModelState, 
    BianconiglioConfig, 
    DataFrameFileExtension, 
    RAGFileExtension,
    RAGAgentState,
    BianconiglioChatResponse
)
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
        updated_models = self.get_xgb_models_info()
        if updated_models and len(updated_models) > 0 and updated_models[0].get("model_id") != "---":
             self.models = updated_models
        return {
            "xgb_model_states": [(state.name, state.value) for state in XgbModelState],
            "xgb_model_ids": [(m["description"], m["model_id"]) for m in self.models],
            "RAG_agent_states": [(state.name, state.value) for state in RAGAgentState],
            "RAG_agent_ids": [(m["description"], m["agent_id"]) for m in self.RAG_agents],
        }
    
    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "config.json")
    
    @property
    def dataframe_extensions(self) -> list[str]:
        return [f".{e.value}" for e in DataFrameFileExtension]
    
    @property
    def rag_extensions(self) -> list[str]:
        return [f".{e.value}" for e in RAGFileExtension]
    
    def load_config(self):
        """loads configuration from Bianconiglioconfig JSON
        """
        self._logger.info(f"Bianconiglio configuration path: {self.config_file_path}")
        self._logger.info(f"Bianconiglio configuration file: {self.config_file}")
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)
                self.config = BianconiglioConfig.FromJSON(config_data)

                self._logger.info("Bianconiglio configuration loaded. URL: %s", self.config.url)

                self.models = self.get_xgb_models_info()
                self.RAG_agents = self.get_RAG_agents_info()

                self._logger.info(f"models: {self.models}")

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
        

    def get_xgb_models_info(self) -> list:
        """Populate the models list with a get request
    
        l'endpoint restituisce: dict {"models_infos": [ModelInfos, ModelInfos, ModelInfos, ...]}
                        oppure: dict {"models_infos": "no models available"}
        """
        if not self.config:
            self._logger.info("config non trovata - 1")
            return []
        
        url = f"{self.config.url}/models"

        try:
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
    
    def get_xgb_model_state(self, model_id: str) -> str | None:
        """Function to get"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return "Error loading Bianconiglio config"
        
        #url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.status_endpoint}"
        url = f"{self.config.url}/models/{model_id}/status"
        print("url status: " + url)
        try:
            response = self.do_get(url, timeout=5)
            print("response status: " + str(response))
            if response and response.get("state", "") in XgbModelState:
                return response.get("state", "")    
            
            return "error getting status"

        except TimeoutError as e:
            self._logger.error(f"Timeout getting logs: {e}")
            return "timeout getting status"
            
        except Exception as e:
            self._logger.error(f"Error getting status: {e}")
            return "error getting status"
        
    def get_log(self, model_id: str):
        """Function to get the logs about a specific model"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return None
        #url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.log_endpoint}"
        url = f"{self.config.url}/models/{model_id}/logs"

        try:
            response = self.do_get(url, timeout=5)       
            
            return response
    
        except TimeoutError as e:
            self._logger.error(f"Timeout getting logs: {e}")
            return None
        
        except Exception as e:
            self._logger.error(f"Error getting logs: {e}")
            return "error getting logs"
        
    def train(self, description: str, data: pd.DataFrame, features: list[str], targets: list[str], url: str | None = None) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        #url = url or f"{self.config.url}{self.config.base_endpoint}{self.config.train_endpoint}"
        url = url or f"{self.config.url}/models/train"

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
                self.models = self.get_xgb_models_info()
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
        
    def retrain(self, model_id: str, new_description: str, data: pd.DataFrame, features: list[str], targets: list[str]):
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        #url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.retrain_endpoint}"
        url = f"{self.config.url}/models{model_id}/retrain"

        response = self.train(new_description, data, features, targets, url)
        return response
        
    def predict(self, model_id: str, data: pd.DataFrame) -> dict:
        if not self.config:
            self._logger.warning("Config is not loaded")
            return {}
        #url = f"{self.config.url}{self.config.base_endpoint}/{model_id}{self.config.predict_endpoint}"
        url = f"{self.config.url}/models/{model_id}/predict"

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
            
        except requests.exceptions.Timeout:
            self._logger.error("TIMEOUT: The prediction is taking too long.")
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
    
        
    def upload_document(self, file_path: str):
        """Load a document from the file manager, reads it and sends it to chord_b via POST.
        Args:
            file_path (str): path of the file to upload, should be in the file manager
        Returns:
            dict: response of POST"""
        
        if not self.config:
            self._logger.warning("Config is not loaded")
            return None

        if FileManager.get_file_extension(file_path) not in self.rag_extensions:
            self._logger.error("File type not supported.")
            return None

        url = f"{self.config.chord_url}/chordB/api/upload" 
        timeout= 60
        try:
            with open(file_path, 'rb') as f:
                data = {
                    "userId": self.config.user,
                    "chatId": self.config.context,
                }

                files = {
                    "file": ( os.path.basename(file_path), f, "application/octet-stream" )
                }

                res = requests.post(
                    url,
                    data=data,
                    files=files,
                    timeout=timeout
                )
                
                if res:
                    self._logger.info(f"Uploading document {file_path} to {url}")

                return

        except requests.exceptions.Timeout:
            self._logger.error("TIMEOUT: The prediction is taking too long.")
            return None
        
        except requests.exceptions.RequestException as e:
            self._logger.error(f"Error during streaming chat: {e}")
            return None  
        
        except Exception as e:
            self._logger.error(f"Error uploading document: {e}")
            return None

    
    def _stream(self, url: str, payload: dict):
        return httpx.stream("POST", url=url, json=payload, timeout=60)
    
    # alternativa streaming
    def stream_chat_with_rag(self, msg: str): # TODO: implementare una classe chat rsponse come per ginos
        """Create the payload with the user quesry and sends i to the agent via POST.
        Args:            
            msg (str): user message
        Returns:            
            dict: response of POST"""
        
        if not self.config:
            self._logger.warning("Config is not loaded")
            return None
        
        url = f"{self.config.chord_url}/api/chat/stream"
        

        self._logger.warning("starting new conversation.")
        
        payload = {
            "message": msg,
            "userId": self.config.user,
            "chatId": self.config.context
        }
        print(f"msg: {msg}, msg_type: {type(msg)}")
        try:
            # with self._stream(url, payload) as response:
            #     for line in response.iter_lines():
            #         if line:
            #             llm_chat_response = json.loads(line)
            #             yield llm_chat_response
            with self._stream(url, payload) as response:
                response.raise_for_status()
                for line in response.iter_lines():
                    if line:
                        decoded_line = line.decode('utf-8').strip() if isinstance(line, bytes) else line.strip()
                        print(f"line response: {decoded_line}")
                        yield BianconiglioChatResponse(decoded_line)

                         
                        
        except requests.exceptions.Timeout:
            self._logger.error("TIMEOUT: The agent is taking too long to answer.")
            return None
        
        except requests.exceptions.RequestException as e:
            self._logger.error(f"Error during streaming chat: {e}")
            return None
        
        except Exception as e:
            self._logger.error(f"Unexpected error during streaming chat: {e}")
            return None
        
        return
    
    def RAG_execute(self) -> bool:
        """function to execute the RAG pipeline with the uploaded documents, should be implemented in chord_b"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return False
        
        url = f"{self.config.chord_url}/execute_rag" # TODO: endpoint inventato da implementare in chord_b

        try:
            response = self.do_post(url, {}, timeout=120)

            if response and response.status_code == 200:
                self._logger.info("RAG execution successful")
                return True
            else:
                self._logger.error(f"RAG execution failed: {response.status_code if response else 'No Response'}")
                if response:
                    self._logger.error(f"Error details: {response.text}")
                return False
        
        except requests.exceptions.Timeout:
            self._logger.error("TIMEOUT: The execution is taking too long.")
            return False
        
        except Exception as e:
            self._logger.error(f"RAG execution error: {e}")
            return False 

    def get_RAG_agents_info(self):
        """Populate the RAG agenst list with a get request
    
        l'endpoint restituisce: dict {"agents": [AgentInfo, AgentInfo, AgentInfo, ...]}
                        oppure: dict {"agents": "no agents available"}
        """
        if not self.config:
            self._logger.info("config non trovata - 1")
            return []
        
        url = f"{self.config.chord_url}api/agents" # TODO: endpoint inventato da implementare in chord_b

        try:
            res = self.do_get(url)

            if not res:
                self._logger.info("No agents found")
                return [{
                    "agent_id": "---",
                    "description": "no agents available"
                    }]
            
            return res["agents"]

        except TimeoutError:
            logging.error(f" timeout api get agents")

        except Exception as e:
            logging.error(f" error while getting agents: {e}")

        return []
    

    def get_RAG_agent_state(self, agent_id: str) -> str | None:
        """Function to get"""
        if not self.config:
            self._logger.warning("Config is not loaded")
            return "Error loading Bianconiglio config"
        
        url = f"{self.config.chord_url}/models/{agent_id}/status" # TODO: endpoint inventato da implementare in chord_b
        print("url status: " + url)
        try:
            response = self.do_get(url, timeout=5)
            print("response status: " + str(response))
            if response and response.get("state", "") in XgbModelState:
                return response.get("state", "")    
            
            return "error getting status"

        except TimeoutError as e:
            self._logger.error(f"Timeout getting logs: {e}")
            return "timeout getting status"
            
        except Exception as e:
            self._logger.error(f"Error getting status: {e}")
            return "error getting status"
        