from dataclasses import dataclass

@dataclass
class CameraWsConfig:
    topic: str
    url: str
    port: int