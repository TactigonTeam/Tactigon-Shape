from paho.mqtt.enums import CallbackAPIVersion, MQTTProtocolVersion
from dataclasses import dataclass


@dataclass
class MQTTConfig:
    broker_url: str
    broker_port: int
    client_id: str
    max_reconnect_count: int = 5
    reconnect_delay: float = 1
    reconnect_rate: int = 2
    protocol_version: MQTTProtocolVersion = MQTTProtocolVersion.MQTTv5
    callback_api_version: CallbackAPIVersion = CallbackAPIVersion.VERSION2


    @classmethod
    def FromJSON(cls, json):
        return cls(
            json["broker_url"],
            json["broker_port"],
            json["client_id"],
            json["max_reconnect_count"],
            json["reconnect_delay"],
            json["reconnect_rate"],
            MQTTProtocolVersion(json["protocol_version"]),
            CallbackAPIVersion(json["callback_api_version"]),
        )
    
    def toJSON(self) -> dict:
        return dict(
            broker_url=self.broker_url,
            broker_port=self.broker_port,
            max_reconnect_count=self.max_reconnect_count,
            reconnect_delay=self.reconnect_delay,
            reconnect_rate=self.reconnect_rate,
            protocol_version=self.protocol_version.value,
            callback_api_version=self.callback_api_version.value,
        )
