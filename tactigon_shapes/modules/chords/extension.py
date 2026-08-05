import logging
import os
import json
import httpx
import requests
from flask import Flask
from typing import Generator, Any

from tactigon_shapes.modules.chords.models import APIResponseStatusEnum, ChatStatus, ChordAgentStateEnum, ChordsLLMConfig, Chat, ChatMessage, ChordFileExtensionEnum
from tactigon_shapes.modules.file_manager.extension import FileManager


class ChordsLLMInterface:
    config_file_path: str | None
    config: ChordsLLMConfig
    chat: Chat | None

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
        return os.path.join(config_file_path, "config.json")

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
            "agent_states": [(state.name, state.value) for state in ChordAgentStateEnum],
        }

    @property
    def rag_extensions(self) -> list[str]:
        return [f".{e.value}" for e in ChordFileExtensionEnum]

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

    def new_chat(self) -> Chat | None:
        res = self._do_get("/api/chat")

        if not res:
            return None

        return Chat(**self._get_data(res))

    def stream(self, content: str) -> Generator[str, Any, None]:
        if not self.chat:
            return None

        req = ChatMessage(
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
        return self._get_status(res) == APIResponseStatusEnum.OK if res else False

    def rag(self) -> bool:
        if not self.chat:
            return False
        
        res = self._do_post(f"/api/chat/{self.chat.chat_id}/rag")

        return self._get_status(res) == APIResponseStatusEnum.OK if res else False

    def chat_status(self) -> ChatStatus | None:
        if not self.chat:
            return None
        
        res = self._do_get(f"/api/chat/{self.chat.chat_id}/status", timeout=5)

        return ChatStatus(**self._get_data(res)) if res else None

    @staticmethod
    def _get_data(res: requests.Response) -> dict:
        return res.json().get("data", {})

    @staticmethod
    def _get_status(res: requests.Response) -> APIResponseStatusEnum:
        return APIResponseStatusEnum(res.json().get("status"))

    def _do_post(self, url: str, payload: dict | None = None, files: dict | None = None, timeout: int = 10) -> requests.Response | None:       
        self._logger.info(f"POST: {url}, {payload}")

        try:
            res = requests.post(
                f"{self.config.url}{url}",
                data=payload,
                files=files,
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

    def _stream(self, url: str, payload: dict):
        return httpx.stream("POST", url=f"{self.config.url}{url}", json=payload)

class ChordsMLInterface:
    pass