from dataclasses import dataclass

@dataclass
class GinosConfig:
    url: str
    model: str

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            url=json["url"],
            model=json["model"]
        )
    
    def toJSON(self) -> dict:
        return dict(
            url=self.url,
            model=self.model
        )