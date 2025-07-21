import dateutil

from enum import Enum
from datetime import datetime
from dataclasses import dataclass, field

from typing import Optional, List

import dateutil.parser

def parse_datetime(date_string: Optional[str]) -> datetime:
    if not date_string:
        return datetime.now()
    
    if date_string.endswith('Z'):
        date_string = date_string[:-1]  # rimuove 'Z'
        date_string = date_string.split('.')[0] + '.' + date_string.split('.')[1][:6]

    return dateutil.parser.isoparse(date_string)

class LLMMessageRole(Enum):
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"
    TOOL = "tool"

@dataclass
class GinosConfig:
    url: str
    model: str

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            url=json.get("url", ""),
            model=json.get("model", ""),
        )
    
    def toJSON(self) -> dict:
        return dict(
            url=self.url,
            model=self.model
        )

@dataclass
class LLMModelPullRequest:
    model: str
    insecure: bool = False
    stream: bool = True

    def toJSON(self) -> dict:
        return dict(
            model=self.model,
            insecure=self.insecure,
            stream=self.stream
        )
    
@dataclass
class LLMModelPullResponse:
    status: str
    digest: str
    total: int
    completed: int
    error: str = ""

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            status=json.get("status", ""),
            digest=json.get("digest", ""),
            total=json.get("total", 0),
            completed=json.get("completed", 0),
            error=json.get("error", "")
        )

@dataclass
class LLMModelDetailsRequest:
    parent_model: str
    format: str
    family: str
    families: List[str]
    parameter_size: str
    quantization_level: str

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            parent_model=json.get("parent_model", ""),
            format=json.get("format", ""),
            family=json.get("family", ""),
            families=json.get("families", ""),
            parameter_size=json.get("parameter_size", ""),
            quantization_level=json.get("quantization_level", ""),
        )
    
@dataclass
class LLMModelRequest:
    name: str
    model: str
    modified_at: datetime
    size: int
    digest: str
    details: Optional[LLMModelDetailsRequest]

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            name=json.get("name", ""),
            model=json.get("model", ""),
            modified_at=parse_datetime(json.get("modified_at", None)),
            size=json.get("size", ""),
            digest=json.get("digest", ""),
            details=LLMModelDetailsRequest.FromJSON(json.get("details", {}))
        )

@dataclass
class LLMModelInfoRequest:
    general_architecture: str
    general_file_type: int
    general_parameter_count: int
    general_quantization_version: int
    llama_attention_head_count: int
    llama_attention_head_count_kv: int
    llama_attention_layer_norm_rms_epsilon:float
    llama_block_count: int
    llama_context_length: int
    llama_embedding_length: int
    llama_feed_forward_length: int
    llama_rope_dimension_count: int
    llama_rope_freq_base: int
    llama_vocab_size: int
    tokenizer_ggml_bos_token_id: int
    tokenizer_ggml_eos_token_id: int
    tokenizer_ggml_merges: List[str]
    tokenizer_ggml_model: str
    tokenizer_ggml_pre: str
    tokenizer_ggml_token_type: List[str]
    tokenizer_ggml_tokens: List[str]

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            general_architecture=json.get("general.architecture", ""),
            general_file_type=json.get("general.file_type", ""),
            general_parameter_count=json.get("general.parameter_count", ""),
            general_quantization_version=json.get("general.quantization_version", ""),
            llama_attention_head_count=json.get("llama.attention.head_count", ""),
            llama_attention_head_count_kv=json.get("llama.attention.head_count_kv", ""),
            llama_attention_layer_norm_rms_epsilon=json.get("llama.attention.layer_norm_rms_epsilon", ""),
            llama_block_count=json.get("llama.block_count", ""),
            llama_context_length=json.get("llama.context_length", ""),
            llama_embedding_length=json.get("llama.embedding_length", ""),
            llama_feed_forward_length=json.get("llama.feed_forward_length", ""),
            llama_rope_dimension_count=json.get("llama.rope.dimension_count", ""),
            llama_rope_freq_base=json.get("llama.rope.freq_base", ""),
            llama_vocab_size=json.get("llama.vocab_size", ""),
            tokenizer_ggml_bos_token_id=json.get("tokenizer.ggml.bos_token_id", ""),
            tokenizer_ggml_eos_token_id=json.get("tokenizer.ggml.eos_token_id", ""),
            tokenizer_ggml_merges=json.get("tokenizer.ggml.merges", ""),
            tokenizer_ggml_model=json.get("tokenizer.ggml.model", ""),
            tokenizer_ggml_pre=json.get("tokenizer.ggml.pre", ""),
            tokenizer_ggml_token_type=json.get("tokenizer.ggml.token_type", ""),
            tokenizer_ggml_tokens=json.get("tokenizer.ggml.tokens", ""),
        )

@dataclass
class LLMModelShowRequest:
    modelfile: str
    paramenter: str
    template: str
    details: LLMModelDetailsRequest
    model_info: LLMModelInfoRequest

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            modelfile=json.get("modelfile", ""),
            paramenter=json.get("paramenter", ""),
            template=json.get("template", ""),
            details=LLMModelDetailsRequest.FromJSON(json.get("details", {})),
            model_info=LLMModelInfoRequest.FromJSON(json.get("model_info", {})),
        )
    
@dataclass
class LLMPromptRequest:
    model: str
    prompt: str
    suffix: str = ""
    images: List[str] = field(default_factory=list)
    format: str = "json"
    template: Optional[str] = None
    context: Optional[str] = None
    stream: bool = True
    keep_alive: str = "5m"

    def to_dict(self) -> dict:
        p = dict(
            model=self.model,
            prompt=self.prompt,
            stream=self.stream,
        )

        if self.context:
            p.update(
                context=self.context
            )
        return p
    
@dataclass
class LLMPromptResponse:
    model: str
    created_at: datetime
    response: str
    done: bool
    done_reason: str
    context: List[int] = field(default_factory=list)
    prompt_eval_count: int = 0
    eval_count: int = 0
    total_duration: int = 0
    load_duration: int = 0
    prompt_eval_duration: int = 0
    eval_duration: int = 0

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            model=json.get("model", ""),
            created_at=parse_datetime(json.get("created_at", None)),
            response=json.get("response", ""),
            done=json.get("done", False),
            done_reason=json.get("done_reason", ""),
            prompt_eval_count=json.get("prompt_eval_count", 0),
            eval_count=json.get("eval_count", 0),
            total_duration=json.get("total_duration", 0),
            load_duration=json.get("load_duration", 0),
            prompt_eval_duration=json.get("prompt_eval_duration", 0),
            eval_duration=json.get("eval_duration", 0),
        )

    
@dataclass
class LLMChatMessageRequest:
    role: LLMMessageRole
    content: str
    images: Optional[str] = None
    tools_call: List[str] = field(default_factory=list)
    keep_alive: str = "5m"

    def toJSON(self) -> dict:
        return dict(
            role=self.role.value,
            content=self.content,
            keep_alive=self.keep_alive,
        )

@dataclass
class LLMChatRequest:
    model: str
    messages: List[LLMChatMessageRequest]
    tools: List[str] = field(default_factory=list)

    def toJSON(self) -> dict:
        c = dict(
            model=self.model,
            messages=[m.toJSON() for m in self.messages],
        )
        return c

@dataclass
class LLMChatMessageResponse:
    role: LLMMessageRole
    content: str

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            role=LLMMessageRole(json.get("role", "tool")),
            content=json.get("content", "")
        )

@dataclass
class LLMChatResponse:
    model: str
    created_at: datetime
    message: LLMChatMessageResponse
    done: bool
    done_reason: str = ""
    prompt_eval_count: int = 0
    eval_count: int = 0
    total_duration: int = 0
    load_duration: int = 0
    prompt_eval_duration: int = 0
    eval_duration: int = 0

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            model=json.get("model", ""),
            created_at=parse_datetime(json.get("created_at", None)),
            message=LLMChatMessageResponse.FromJSON(json.get("message", {})),
            done=json.get("done", False),
            done_reason=json.get("done_reason", ""),
            prompt_eval_count=json.get("prompt_eval_count", 0),
            eval_count=json.get("eval_count", 0),
            total_duration=json.get("total_duration", 0),
            load_duration=json.get("load_duration", 0),
            prompt_eval_duration=json.get("prompt_eval_duration", 0),
            eval_duration=json.get("eval_duration", 0),
        )
