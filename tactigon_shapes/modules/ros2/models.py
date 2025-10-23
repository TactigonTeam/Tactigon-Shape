from rclpy.node import QoSProfile

from dataclasses import dataclass, field
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
    
@dataclass
class Ros2Publisher:
    topic: str
    message_type: Any
    qos_profile: QoSProfile | int = 10
    
@dataclass
class Ros2Subscription:
    topic: str
    function: str
    payload_reference: str
    message_type: Any
    qos_profile: QoSProfile | int = 10

@dataclass
class Ros2Config:
    node_name: str
    publisher: list[Ros2Publisher] = field(default_factory=list)
    subscriptions: list[Ros2Subscription] = field(default_factory=list)