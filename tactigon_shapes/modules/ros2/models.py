from dataclasses import dataclass
from enum import Enum
from typing import Any

@dataclass
class RosMessage:
    topic: str
    msg: Any

class NodeActions(Enum):
    ADD_PUBLISHER = "add_publisher"
    ADD_SUBSCRIPTION = "add_subscriber"
    PUBLISH = "publish"
    UNSUBSCRIBE = "unsubscribe"

@dataclass
class NodeAction:
    action: NodeActions
    payload: dict

    @classmethod
    def AddPubblisher(cls, payload: dict):
        return cls(
            NodeActions.ADD_PUBLISHER,
            payload
        )
    
    @classmethod
    def AddSubscription(cls, payload: dict):
        return cls(
            NodeActions.ADD_SUBSCRIPTION,
            payload
        )
    
    @classmethod
    def Publish(cls, payload: dict):
        return cls(
            NodeActions.PUBLISH,
            payload
        )
    
    @classmethod
    def Unsubscribe(cls, payload: dict):
        return cls(
            NodeActions.UNSUBSCRIBE,
            payload
        )