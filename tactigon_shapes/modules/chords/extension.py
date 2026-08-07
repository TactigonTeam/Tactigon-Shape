import logging
import os
import json
import httpx
import requests
import pandas as pd
from flask import Flask
from typing import Generator, Any

from tactigon_shapes.modules.chords.models import (
    ChordsLLMAPIResponseStatusEnum, 
    ChordsLLMChatStatus, 
    ChordsLLMAgentStateEnum, 
    ChordsLLMConfig, 
    ChordsLLMChat, 
    ChordsLLMChatMessage, 
    ChordsLLMFileExtensionEnum,

    ChordsMLConfig,
    ChordsMLDFFileExtension,
    ChordsMLModelInfo,
    ChordsMLModelStateEnum,


)
from tactigon_shapes.modules.file_manager.extension import FileManager


class ChordsLLMInterface:
    config_file_path: str | None
    config: ChordsLLMConfig
    chat: ChordsLLMChat | None

    def __init__(self, config_file_path: str | None = None, app: Flask | None = None):
        self._logger = logging.getLogger(ChordsLLMInterface.__name__)
        self.config_file_path = config_file_path

        if config_file_path:
            self.config = self.load_config(config_file_path)
        else:
            self.config = self.default_config()
        self.chat = None
    
        if app:
            self.init_app(app)

    @staticmethod
    def config_file(config_file_path: str) -> str:
        return os.path.join(config_file_path, "llm_config.json")

    @staticmethod
    def load_config(config_file_path: str):
        if os.path.exists(ChordsLLMInterface.config_file(config_file_path)):
            with open(ChordsLLMInterface.config_file(config_file_path), "r") as f:
                config_data = json.load(f)
                return ChordsLLMConfig.FromJSON(config_data)
        else:
            return ChordsLLMInterface.default_config()

    @staticmethod
    def default_config() -> ChordsLLMConfig:
        return ChordsLLMConfig(
            url="http://localhost:8080",
        )

    @staticmethod
    def get_shape_blocks():
        return {
            "agent_states": [(state.name, state.value) for state in ChordsLLMAgentStateEnum],
            "valid_extensions": [ext.value for ext in ChordsLLMFileExtensionEnum]
        }

    @property
    def rag_extensions(self) -> list[str]:
        return [f".{e.value}" for e in ChordsLLMFileExtensionEnum]

    def save_config(self):
        if self.config_file_path:
            with open(self.config_file(self.config_file_path), "w") as f:
                json.dump(self.config.model_dump_json(), f)

    def init_app(self, app: Flask):
        app.extensions[ChordsLLMInterface.__name__] = self

    def init(self):
        self.chat = self.new_chat()
        self._logger.info(f"Got new chat context {self.chat}")

    def deinit(self):
        self.chat = None
        self._logger.info(f"Removed chat context ({self.chat})")

    def new_chat(self) -> ChordsLLMChat | None:
        res = self._do_get("/api/chat")

        if not res:
            return None

        return ChordsLLMChat(**self._get_data(res))

    def stream(self, content: str) -> Generator[str, Any, None]:
        if not self.chat:
            return None

        req = ChordsLLMChatMessage(
            chat_id=self.chat.chat_id,
            user_id=self.chat.user_id,
            content=content
        )

        with self._stream(f"/api/chat/{self.chat.chat_id}/stream", req.model_dump()) as response:
            for line in response.iter_lines():
                self._logger.info(f"Got stream: {line}")
                yield line

    def upload(self, file_path: str) -> bool:
        if FileManager.get_file_extension(file_path) not in self.rag_extensions:
            self._logger.error("File type not supported.")
            return False

        if not self.chat:
            return False

        data = {
            "user_id": self.chat.user_id,
        }

        with open(file_path, 'rb') as f:
            files = {
                "file": ( os.path.basename(file_path), f, "application/octet-stream" )
            }

            res = self._do_post(f"/api/chat/{self.chat.chat_id}/upload", data, files)
        return self._get_status(res) == ChordsLLMAPIResponseStatusEnum.OK if res else False

    def rag(self) -> bool:
        if not self.chat:
            return False
        
        res = self._do_post(f"/api/chat/{self.chat.chat_id}/rag")

        return self._get_status(res) == ChordsLLMAPIResponseStatusEnum.OK if res else False

    def chat_status(self) -> ChordsLLMChatStatus | None:
        if not self.chat:
            return None
        
        res = self._do_get(f"/api/chat/{self.chat.chat_id}/status", timeout=5)

        return ChordsLLMChatStatus(**self._get_data(res)) if res else None

    @staticmethod
    def _get_data(res: requests.Response) -> dict:
        return res.json().get("data", {})

    @staticmethod
    def _get_status(res: requests.Response) -> ChordsLLMAPIResponseStatusEnum:
        return ChordsLLMAPIResponseStatusEnum(res.json().get("status"))

    def _do_post(self, url: str, payload: dict | None = None, files: dict | None = None, timeout: int = 10) -> requests.Response | None:       
        self._logger.info(f"POST: {url}, {payload}")

        try:
            res = requests.post(
                f"{self.config.url}{url}",
                data=payload,
                files=files,
                timeout=timeout,
                verify=False
            )
            res.raise_for_status()
            
            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        except requests.exceptions.Timeout:
            self._logger.error(f"POST {url} Timeout expired (server hanging)")
        except Exception as e:
            self._logger.warning("POST %s failed: %s", url, e)
                
        return None

    def _do_get(self, url: str, timeout: int = 5) -> requests.Response | None:
        try:
            res = requests.get(f"{self.config.url}{url}", timeout=timeout, verify=False)
            res.raise_for_status()

            self._logger.info(f"GET %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None

    def _stream(self, url: str, payload: dict):
        return httpx.stream("POST", url=f"{self.config.url}{url}", json=payload, verify=False, follow_redirects=True)

class ChordsMLInterface:
    config: ChordsMLConfig

    def __init__(self, config_file_path: str | None = None, app: Flask | None = None):
        self._logger = logging.getLogger(ChordsMLInterface.__name__)
        self.config_file_path = config_file_path

        if config_file_path:
            self.config = self.load_config(config_file_path)
        else:
            self.config = self.default_config()
    
        if app:
            self.init_app(app)

    @staticmethod
    def config_file(config_file_path: str) -> str:
        return os.path.join(config_file_path, "ml_config.json")

    @staticmethod
    def load_config(config_file_path: str):
        if os.path.exists(ChordsMLInterface.config_file(config_file_path)):
            with open(ChordsMLInterface.config_file(config_file_path), "r") as f:
                config_data = json.load(f)
                return ChordsMLConfig.FromJSON(config_data)
        else:
            return ChordsMLInterface.default_config()

    @staticmethod
    def default_config() -> ChordsMLConfig:
        return ChordsMLConfig()

    @property
    def dataframe_extensions(self) -> list[str]:
        return [f".{e.value}" for e in ChordsMLDFFileExtension]

    def save_config(self):
        if self.config_file_path:
            with open(self.config_file(self.config_file_path), "w") as f:
                json.dump(self.config.model_dump_json(), f)

    def init_app(self, app: Flask):
        app.extensions[ChordsMLInterface.__name__] = self

        self.get_models_info()
    
    def get_shape_blocks(self):
        return {
            "model_states": [(state.name, state.value) for state in ChordsMLModelStateEnum],
            "valid_extensions": [ext.value for ext in ChordsMLDFFileExtension],
            "models": [(m.description, m.model_id) for m in self.models] 
                if self.models else [("No model available yet", "---")]
        }
        
    def get_models_info(self):
        res = self._do_get("/models")

        if res:
            self._logger.info(f"Got models {res.json()}")
            self.models = [ChordsMLModelInfo.FromJSON(m) for m in res.json().get("models_infos", [])]
            self._logger.info(f"Loaded {len(self.models)} models from Chords")
        else:
            self.models = []
    
    def get_model_state(self, model_id: str) -> ChordsMLModelStateEnum | None:   
        res = self._do_get(f"/models/{model_id}/status")
        
        return ChordsMLModelStateEnum(res.json().get("state", "ERROR")) if res else None

    def get_log(self, model_id: str):
        res = self._do_get(f"/models/{model_id}/logs", timeout=5)       
            
        return res.json() if res else {}
        
    def train(self, description: str, data: pd.DataFrame, features: list[str], targets: list[str]) -> dict:
        return self._train(
            url="/models/train",
            description=description,
            data=data,
            features=features,
            targets=targets
        )
        
    def retrain(self, model_id: str, description: str, data: pd.DataFrame, features: list[str], targets: list[str]):
        return self._train(
            url=f"/models{model_id}/retrain",
            description=description,
            data=data,
            features=features,
            targets=targets
        )

    def _train(self, url: str, description: str, data: pd.DataFrame, features: list[str], targets: list[str]) -> dict:
        self._logger.info(f"Train {type(data)} data: {data}")
        self._logger.info(f"Features: {features}")
        self._logger.info(f"Targets: {targets}")
        
        payload = {
            "description": description,
            "data": data.to_dict(orient="records"),
            "features": features,
            "targets": targets
        }

        res = self._do_post(url, payload=payload)

        if res and res.status_code == 200:
            self._logger.info("Train successful")
            return res.json()
        elif res:
            self._logger.error(f"Error from server during training. Code: {res.status_code}")
            return res.json()
        else:
            return {}
        
    def predict(self, model_id: str, data: pd.DataFrame) -> dict:        
        self._logger.info(f"Predit {type(data)} data: {data}")

        payload = {
            "data": data.to_dict(orient="records"),
        }

        res = self._do_post(f"/models/{model_id}/predict", payload)

        if res and res.status_code == 200:
            self._logger.info("Predict successful")
            return res.json()
        else:
            self._logger.error(f"Predict failed: {res.status_code if res else 'No Response'}")
            if res:
                self._logger.error(f"Error details: {res.text}")

        return {}     

    def file_to_dataframe(self,file_path: str) -> pd.DataFrame | None:
        if FileManager.get_file_extension(file_path) not in self.dataframe_extensions:
            self._logger.error("File type not supported.")
            return None

        df = None

        try:
            if file_path.endswith('.csv'):
                df = pd.read_csv(file_path)
            elif file_path.endswith('.json'):
                df = pd.read_json(file_path)
            
        except Exception as e:
            self._logger.error("Cannot read file into dataframe. %s", e.with_traceback)
            
        return df
    
    def get_dataframe(self, file_path: str) -> pd.DataFrame | None:
        df = self.file_to_dataframe(file_path)
        
        if df is None:
            self._logger.error(f"Cannot get dataframe from file {file_path}")

        return df

    def _do_post(self, url: str, payload: dict | None = None, files: dict | None = None, timeout: int = 10) -> requests.Response | None:       
        self._logger.info(f"POST: {url}, {payload}")

        try:
            if files is not None:
                res = requests.post(
                    f"{self.config.url}{url}",
                    data=payload,
                    files=files,
                    timeout=timeout
                )
            else:
                res = requests.post(
                    f"{self.config.url}{url}",
                    json=payload,
                    timeout=timeout
                )
            res.raise_for_status()
            
            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        except requests.exceptions.Timeout:
            self._logger.error(f"POST {url} Timeout expired (server hanging)")
        except Exception as e:
            self._logger.warning("POST %s failed: %s", url, e)
                
        return None

    def _do_get(self, url: str, timeout: int = 5) -> requests.Response | None:
        try:
            res = requests.get(f"{self.config.url}{url}", timeout=timeout)
            res.raise_for_status()

            self._logger.info(f"GET %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None