import json
import logging
import requests
import httpx

from flask import Flask

from typing import Iterator, Optional, List

from .models import LLMChatResponse, LLMModelPullRequest, LLMModelPullResponse, LLMModelRequest, LLMModelShowRequest, LLMPromptRequest, LLMChatRequest, LLMPromptResponse

APPLICATION_JSON = 'application/json'

HEADERS = {
    "accept": APPLICATION_JSON,
}

class GinosInterface:
    _url: str
    _model: str

    _version: Optional[str]
    _model_list: List[LLMModelRequest]

    def __init__(self, url: str, model: str):
        self._url = url if url[-1] == "/" else f"{url}/"
        self._model = model

        self._version = self.get_version()
        self.get_models()

    @property
    def url(self):
        return f"{self._url}api"
    
    @property
    def connected(self) -> bool:
        return self._version is not None
    
    @property
    def models(self) -> List[LLMModelRequest]:
        return self._model_list
    
    @property
    def model(self) -> str:
        return self._model

    def get_version(self) -> Optional[str]:
        resp = self._get("version")

        return resp.get("version", None)
    
    def get_models(self):
        resp = self._get("tags")

        models_resp = resp.get("models", None)
        self._model_list = []
        
        if models_resp:
            self._model_list = [LLMModelRequest.FromJSON(m) for m in models_resp]
    
    def find_model(self, model_name: str) -> Optional[LLMModelRequest]:
        return next((m for m in self._model_list if m.name == model_name), None)

    def show_model(self, model_name: str) -> Optional[LLMModelShowRequest]:
        response_json = {}
        try:
            res = self._post("show", dict(model=model_name))
            response_json = res.json()
        except Exception as e:
            logging.error(e)
            return None

        if response_json.get("error", False):
            logging.error(response_json.get("error"))
            return None
        
        return LLMModelShowRequest.FromJSON(response_json)

    def pull_model(self, model_name: str) -> Iterator[LLMModelPullResponse]:
        try:
            with self._stream("pull", LLMModelPullRequest(model_name).toJSON()) as res:
                for line in res.iter_lines():
                    llm_model_pull_response = LLMModelPullResponse.FromJSON(json.loads(line))
                    logging.debug("Response line: %s", llm_model_pull_response)
                    yield llm_model_pull_response

        except Exception as e:
            logging.error(e)
            return None
        
        self.get_models()

        return

    def remove_model(self, model_name: str) -> bool:
        return self._delete("delete", dict(model=model_name)) == 200

    def prompt(self, prompt: LLMPromptRequest) -> Iterator[LLMPromptResponse]:
        if not self.find_model(prompt.model):
            self.pull_model(prompt.model)

        logging.info("Sending prompt %s", prompt)
        try:
            with self._stream("generate", prompt.to_dict()) as res:
                for line in res.iter_lines():
                    llm_prompt_response = LLMPromptResponse.FromJSON(json.loads(line))
                    logging.debug("Response line: %s", llm_prompt_response)
                    yield llm_prompt_response
                
        except Exception as e:
            logging.error(e)
            return None

        return
    
    def chat(self, chat: LLMChatRequest) -> Iterator[LLMChatResponse]:
        if not self.find_model(chat.model):
            self.pull_model(chat.model)

        logging.info("Sending chat %s", chat)
        try:
            with self._stream("chat", chat.toJSON()) as res:
                for line in res.iter_lines():
                    llm_chat_response = LLMChatResponse.FromJSON(json.loads(line))
                    logging.debug("Response: %s", llm_chat_response)
                    yield llm_chat_response
        except Exception as e:
            logging.error(e)
            return None

        return

    def _get(self, path: str) -> dict:
        url = f"{self.url}/{path}"

        try:
            res = httpx.get(url, headers=HEADERS)
            return res.json()
        except Exception as e:
            logging.error(e)

        return {}
    
    def _post(self, path: str, payload: dict):
        url = f"{self.url}/{path}"
        return httpx.post(url, headers=HEADERS, json=payload, timeout=None)

    def _stream(self, path: str, payload: dict):
        url = f"{self.url}/{path}"
        return httpx.stream("POST", url, headers=HEADERS, json=payload, timeout=None)
    
    def _delete(self, path: str, payload: dict) -> int:
        url = f"{self.url}/{path}"

        try:
            res = requests.delete(url, headers=HEADERS, json=payload)
            return res.status_code
        except Exception as e:
            logging.error(e)

        return 500