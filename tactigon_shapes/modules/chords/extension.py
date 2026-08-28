import logging
import os
import json
import httpx
import requests
import urllib3
import pandas as pd
from flask import Flask
from typing import Generator, Any

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

from tactigon_shapes.modules.chords.models import (
    ChordLLMApiResponseStatusEnum, 
    ChordLLMChatStatus, 
    ChordLLMAgentStateEnum, 
    ChordLLMConfig, 
    ChordLLMChat, 
    ChordLLMChatStream, 
    ChordLLMFileExtensionEnum,
    ChordLLMPromptSchema,

    ChordsMLConfig,
    ChordsMLDFFileExtension,
    ChordsMLModelInfo,
    ChordsMLModelStateEnum,

    
)
from tactigon_shapes.modules.file_manager.extension import FileManager

APPLICATION_JSON = 'application/json'

class ChordLLMInterface:
    config_file_path: str
    config: ChordLLMConfig
    prompts: list[ChordLLMPromptSchema]
    chat: ChordLLMChat | None
    access_token: str | None
    refresh_token: str | None

    def __init__(self, config_file_path: str, app: Flask | None = None):
        self._logger = logging.getLogger(ChordLLMInterface.__name__)
        self.config_file_path = config_file_path

        self.chat = None
        self.access_token = None
        self.refresh_token = None
        self.prompts = []

        self.load_config()

        self._logger.info("Created!")
    
        if app:
            self.init_app(app)

    @property
    def config_file(self) -> str:
        return os.path.join(self.config_file_path, "llm_config.json")

    @property
    def rag_extensions(self) -> list[str]:
        return [f".{e.value}" for e in ChordLLMFileExtensionEnum]

    @property
    def configured(self) -> bool:
        return False if self.config is None else True
   

    @property
    def token(self) -> str:
        return self.access_token if self.access_token else ""
    
    def load_config(self):
        if os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                config_data = json.load(f)
                self.config = ChordLLMConfig.FromJSON(config_data)
                if self.config.is_valid():
                    self.prompts = self.get_prompts()
        else:
            self.config = ChordLLMConfig()

    def get_shape_blocks(self):
        return {
            "agent_states": [(state.name, state.value) for state in ChordLLMAgentStateEnum],
            "valid_extensions": [ext.value for ext in ChordLLMFileExtensionEnum],
            "prompts": [(p.name, p.id) for p in self.prompts]
        }
    
    def save_config(self, config: ChordLLMConfig):
        """save config to remain configurated

        Args:
            config (ZionConfig): the Zion Configuration
        """
        if not os.path.exists(self.config_file_path):
            os.makedirs(self.config_file_path)

        with open(self.config_file, "w") as f:
            json.dump(config.toJSON(), f, indent=2)

        self._logger.info("Zion configuration saved.")
        self.load_config()

    def reset_config(self):
        """ remove config file and reloads it
        """
        if os.path.exists(self.config_file_path) and os.path.exists(self.config_file):
            os.remove(self.config_file)

        self.load_config()

    def init_app(self, app: Flask):
        app.extensions[ChordLLMInterface.__name__] = self

    def init(self, prompt: str | None):
        self.chat = self.new_chat(prompt)

    def deinit(self):
        self.chat = None
        self._logger.info(f"Removed chat context ({self.chat})")

    def login(self, username: str, password: str) -> bool:
        resp = self._do_post(
            "/api/auth/local", 
            payload={
                "username": username,
                "password": password,
            },
            auth=False
        )

        if not resp:
            return False

        if self._get_status(resp) != ChordLLMApiResponseStatusEnum.OK:
            return False

        data = self._get_data(resp)
        self.access_token = data.get("access_token", None)
        self.refresh_token = data.get("refresh_token", None)

        return True

    def logout(self):
        resp = self._do_post("/api/auth/logout")

        if resp and self._get_status(resp) == ChordLLMApiResponseStatusEnum.OK:
            self._logger.info("User logged out!")
            return

        self._logger.warning(f"Could not log out. Error: {self._get_error(resp) if resp else "No response"}")
        
    def new_chat(self, prompt: str | None = None) -> ChordLLMChat | None:
        res = self._do_post(
            "/api/chat/",
            payload={
                "prompt": prompt
            }
        )

        if not res:
            return None

        return ChordLLMChat(**self._get_data(res))

    def stream(self, content: str) -> Generator[str, Any, None]:
        if not self.chat:
            return

        req = ChordLLMChatStream(
            content=content
        )

        url = f"/api/chat/{self.chat.chat_id}/stream"
        payload = req.model_dump()

        for attempt in range(2):
            with self._stream(url, payload) as response:
                if response.status_code == 401 and attempt == 0:
                    self._logger.info("Stream got 401, trying to re-login")
                    if not self.login(self.config.username, self.config.password):
                        return
                    continue

                response.raise_for_status()

                for line in response.iter_lines():
                    self._logger.info(f"Got stream: {line}")
                    yield line
            return

    def upload(self, file_path: str) -> bool:
        if FileManager.get_file_extension(file_path) not in self.rag_extensions:
            self._logger.error("File type not supported.")
            return False

        if not self.chat:
            return False

        with open(file_path, 'rb') as f:
            files = {
                "file": ( os.path.basename(file_path), f, "application/octet-stream" )
            }

            res = self._do_post(f"/api/chat/{self.chat.chat_id}/upload", files=files)
        return self._get_status(res) == ChordLLMApiResponseStatusEnum.OK if res else False

    def rag(self) -> bool:
        if not self.chat:
            return False
        
        res = self._do_post(f"/api/chat/{self.chat.chat_id}/rag")

        return self._get_status(res) == ChordLLMApiResponseStatusEnum.OK if res else False

    def chat_status(self) -> ChordLLMChatStatus | None:
        if not self.chat:
            return None
        
        res = self._do_get(f"/api/chat/{self.chat.chat_id}/status", timeout=5)

        return ChordLLMChatStatus(**self._get_data(res)) if res else None

    def get_prompts(self):
        res = self._do_get("/api/prompt")

        prompts = []
        if res:
            prompts = [ChordLLMPromptSchema.model_validate(p) for p in self._get_data(res).get("prompts", [])]

        self._logger.info(f"Loaded {len(prompts)} prompts")

        return prompts

    @staticmethod
    def _get_data(res: requests.Response) -> dict:
        return res.json().get("data", {})

    @staticmethod
    def _get_status(res: requests.Response) -> ChordLLMApiResponseStatusEnum:
        return ChordLLMApiResponseStatusEnum(res.json().get("status"))

    @staticmethod
    def _get_error(res: requests.Response) -> str:
        return res.json().get("error", "")

    def _do_post(self, url: str, payload: dict | None = None, files: dict | None = None, auth: bool = True, timeout: int = 10) -> requests.Response | None:       
        headers = {
            "accept": APPLICATION_JSON,
        }

        if auth and self.access_token:
            headers["Authorization"] = f"Bearer {self.access_token}"

        try:
            if files is not None:
                res = requests.post(
                    f"{self.config.url}{url}",
                    data=payload,
                    files=files,
                    timeout=timeout,
                    verify=False,
                    headers=headers
                )
            else:
                res = requests.post(
                    f"{self.config.url}{url}",
                    json=payload,
                    timeout=timeout,
                    verify=False,
                    headers=headers
                )

            self._logger.info(f"POST: {url}, payload: {payload}. Response {res.status_code}")

            if res.status_code == 401:
                if not self.login(self.config.username, self.config.password):
                    return None
                
                return self._do_post(url, payload, files, auth, timeout)

            res.raise_for_status()
            
            self._logger.debug("POST %s payload: %s response: %s", url, payload, res.status_code)
            return res
        except requests.exceptions.Timeout:
            self._logger.error(f"POST {url} Timeout expired (server hanging)")
        except Exception as e:
            self._logger.warning("POST %s failed: %s", url, e)
                
        return None

    def _do_get(self, url: str, auth: bool = True, timeout: int = 5) -> requests.Response | None:

        headers = {}

        if auth and self.access_token:
            headers["Authorization"] = f"Bearer {self.access_token}"

        try:
            res = requests.get(
                f"{self.config.url}{url}", 
                headers=headers,
                timeout=timeout, 
                verify=False
            )

            if res.status_code == 401:
                if not self.login(self.config.username, self.config.password):
                    return None
                
                return self._do_get(url, auth, timeout)
            
            res.raise_for_status()

            self._logger.info(f"GET %s response: %s", url, res.status_code)
            return res

        except Exception as e:
            self._logger.warning("GET %s failed: %s", url, e)
            return None

    def _stream(self, url: str, payload: dict):
        return httpx.stream(
            "POST", 
            url=f"{self.config.url}{url}", 
            json=payload, 
            verify=False, 
            follow_redirects=True,
            headers={
                "Authorization": f"Bearer {self.access_token}"
            }
        )

class ChordMLInterface:
    config: ChordsMLConfig

    def __init__(self, config_file_path: str | None = None, app: Flask | None = None):
        self._logger = logging.getLogger(ChordMLInterface.__name__)
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
        if os.path.exists(ChordMLInterface.config_file(config_file_path)):
            with open(ChordMLInterface.config_file(config_file_path), "r") as f:
                config_data = json.load(f)
                return ChordsMLConfig.FromJSON(config_data)
        else:
            return ChordMLInterface.default_config()

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
        app.extensions[ChordMLInterface.__name__] = self

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