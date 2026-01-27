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

from pathlib import Path
import json
import logging
import requests
import httpx
import time
import pandas

from typing import Iterator

from tactigon_shapes.modules.file_manager.extension import FileManager
from tactigon_shapes.modules.ginos.models import LLMChatResponse, LLMModelPullRequest, \
    LLMModelPullResponse, LLMModelRequest, LLMModelShowRequest, \
        LLMPromptRequest, LLMChatRequest, LLMPromptResponse, \
        DataFrameFileExtension

APPLICATION_JSON = 'application/json'

HEADERS = {
    "accept": APPLICATION_JSON,
}

class GinosInterface:
    _url: str
    _model: str

    _version: str | None
    _model_list: list[LLMModelRequest]
    _logger: logging.Logger
    _dataframe: pandas.DataFrame | None

    def __init__(self, url: str, model: str):
        self._url = url if url[-1] == "/" else f"{url}/"
        self._model = model
        self._logger = logging.getLogger(GinosInterface.__name__)
        self._dataframe = None

        retries = 0
        while retries < 6:
            self._version = self.get_version()
            if self._version is not None:
                break
            
            self._logger.warning("Retrying Ollama connection")
            time.sleep(0.5)
            retries += 1

        self.get_models()

    @property
    def url(self):
        return f"{self._url}api"
    
    @property
    def connected(self) -> bool:
        return self._version is not None
    
    @property
    def models(self) -> list[LLMModelRequest]:
        return self._model_list
    
    @property
    def model(self) -> str:
        return self._model
    
    @property
    def dataframe_extensions(self) -> list[str]:
        return [e.value for e in DataFrameFileExtension]

    def get_version(self) -> str | None:
        resp = self._get("version")

        return resp.get("version", None)
    
    def get_models(self):
        resp = self._get("tags")

        models_resp = resp.get("models", None)
        self._model_list = []
        
        if models_resp:
            self._model_list = [LLMModelRequest.FromJSON(m) for m in models_resp]
    
    def find_model(self, model_name: str) -> LLMModelRequest | None:
        return next((m for m in self._model_list if m.name == model_name), None)

    def show_model(self, model_name: str) -> LLMModelShowRequest | None:
        response_json = {}
        try:
            res = self._post("show", dict(model=model_name))
            response_json = res.json()
        except Exception as e:
            self._logger.error(e)
            return None

        if response_json.get("error", False):
            self._logger.error(response_json.get("error"))
            return None
        
        return LLMModelShowRequest.FromJSON(response_json)

    def pull_model(self, model_name: str) -> Iterator[LLMModelPullResponse]:
        try:
            with self._stream("pull", LLMModelPullRequest(model_name).toJSON()) as res:
                for line in res.iter_lines():
                    llm_model_pull_response = LLMModelPullResponse.FromJSON(json.loads(line))
                    self._logger.debug("Response line: %s", llm_model_pull_response)
                    yield llm_model_pull_response

        except Exception as e:
            self._logger.error(e)
            return None
        
        self.get_models()

        return

    def remove_model(self, model_name: str) -> bool:
        return self._delete("delete", dict(model=model_name)) == 200

    def prompt(self, prompt: LLMPromptRequest) -> Iterator[LLMPromptResponse]:
        if not self.find_model(prompt.model):
            self.pull_model(prompt.model)

        self._logger.info("Sending prompt %s", prompt)
        try:
            with self._stream("generate", prompt.to_dict()) as res:
                for line in res.iter_lines():
                    llm_prompt_response = LLMPromptResponse.FromJSON(json.loads(line))
                    self._logger.debug("Response line: %s", llm_prompt_response)
                    yield llm_prompt_response
                
        except Exception as e:
            self._logger.error(e)
            return None

        return
    
    def chat(self, chat: LLMChatRequest) -> Iterator[LLMChatResponse]:
        if not self.find_model(chat.model):
            self.pull_model(chat.model)

        self._logger.info("Sending chat %s", chat)
        try:
            with self._stream("chat", chat.toJSON()) as res:
                for line in res.iter_lines():
                    llm_chat_response = LLMChatResponse.FromJSON(json.loads(line))
                    self._logger.debug("Response: %s", llm_chat_response)
                    yield llm_chat_response
        except Exception as e:
            self._logger.error(e)
            return None

        return

    def _get(self, path: str) -> dict:
        url = f"{self.url}/{path}"

        try:
            res = httpx.get(url, headers=HEADERS)
            return res.json()
        except Exception as e:
            self._logger.error(e)

        return {}
    
    def _post(self, path: str, payload: dict):
        url = f"{self.url}/{path}"
        return httpx.post(url, headers=HEADERS, json=payload, timeout=None)

    def _stream(self, path: str, payload: dict):
        url = f"{self.url}/{path}"
        return httpx.stream("POST", url, headers=HEADERS, json=payload, timeout=60)
    
    def _delete(self, path: str, payload: dict) -> int:
        url = f"{self.url}/{path}"

        try:
            res = requests.delete(url, headers=HEADERS, json=payload)
            return res.status_code
        except Exception as e:
            self._logger.error(e)

        return 500
    
    def file_to_dataframe(self,file_path: str) -> pandas.DataFrame | None:

        if FileManager.get_file_extension(file_path) not in self.dataframe_extensions:
            return None

        try:
            df = None

            if file_path.endswith('.csv'):
                df = pandas.read_csv(file_path)

            elif file_path.endswith('.json'):
                df = pandas.read_json(file_path)

            # elif file_path.endswith(('.txt', '.md')):
            #     with open(file_path, 'r', encoding='utf-8') as f:
            #         lines = [line.strip() for line in f.readlines() if line.strip()]

            #     return pandas.DataFrame(lines, columns=['content'])
            
            return df
        except Exception as e:
            self._logger.error("Cannot read file into dataframe. %s", e.with_traceback)
            
        return None

    def add_file_to_context(self, file_path: str):
            
        new_df = self.file_to_dataframe(file_path)
        
        if new_df is None:
            self._logger.error(f"Impossibile aggiungere il file: {file_path}")
            return False

        if self._dataframe is None:
            # È il primo file: il master DataFrame diventa questo
            self._dataframe = new_df
        else:
            # Concateniamo i dati nuovi a quelli vecchi
            # 'ignore_index=True' serve per non avere indici duplicati
            self._dataframe = pandas.concat([self._dataframe, new_df], ignore_index=True)
        
        self._logger.debug("\n" + self._dataframe.to_string())
        self._logger.info(f"Added {file_path} to context")
        return True

    def clear_context(self):
        self._dataframe = None
        self._logger.info("INFO: dataframe pulito")
