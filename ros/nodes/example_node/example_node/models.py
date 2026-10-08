from dataclasses import dataclass

@dataclass
class ExampleConfig:
    debug: bool = False
    subscription_topic: str = "example_subscription"
    publish_topic: str = "example_publish"

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            json.get("debug", False),
            json.get("subscription_topic", "example_subscription"),
            json.get("publish_topic", "example_publish"),
        )
    
    def toJSON(self) -> dict:
        return dict(
            debug=self.debug,
            subscription_topic=self.subscription_topic,
            publish_topic=self.publish_topic,
        )