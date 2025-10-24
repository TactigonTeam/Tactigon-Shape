import rclpy
import time
from rclpy.node import Node, QoSProfile
from std_msgs.msg import String
from threading import Thread, Event as ThreadEvent
from multiprocessing import Process, Queue, Event, Pipe
from queue import Queue, Empty
from functools import wraps
from flask import Flask

from typing import Callable, Optional, Any

from tactigon_shapes.modules.ros2.models import Ros2Config, RosMessage, NodeAction, NodeActions

class ShapeNode(Node):
    TICK: float = 0.02

    def __init__(self):
        Node.__init__(self, "tactigon_shape")

    def add_publisher(self, topic: str, message_type: Any, qos_profile: QoSProfile | int):
        publisher = self.create_publisher(message_type, topic, qos_profile)

    def add_subscription(self, topic: str, fn: Callable[[RosMessage], None], message_type: Any, qos_profile: QoSProfile | int):
        self.create_subscription(
            message_type, 
            topic, 
            self._callback(topic, fn), 
            qos_profile
        )

    def publish(self, topic: str, message_type: Any, msg: Any):
        publisher = next((p for p in self.publishers if p.topic == topic), None)

        if publisher:
            message = message_type()
            message.data = msg
            publisher.publish(message)

    def unsubscribe(self, topic: str):
        subscription = next((s for s in self.subscriptions if s.topic == topic), None)

        if subscription:
            self.destroy_subscription(subscription)

    def _callback(self, topic: str, fn: Callable[[RosMessage], None]):
        @wraps(fn)
        def wrapper(msg):
            result = fn(RosMessage(topic, msg))
            return result
        return wrapper

class Ros2Process(Process):
    def __init__(self):
        Process.__init__(self, daemon=True)
        self._stop_event = Event()
        self._in, self._out = Pipe()

    def run(self):
        rclpy.init()
        try:
            node = ShapeNode()

            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0)
                if self._in.poll():
                    node_action: NodeAction = self._in.recv()
                    action = node_action.action
                    payload = node_action.payload

                    if action == NodeActions.ADD_PUBLISHER:
                        node.add_publisher(
                            payload.get("topic", ""), 
                            payload.get("message_type", String),
                            payload.get("qos_profile", 10)
                        )
                    elif action == NodeActions.ADD_SUBSCRIPTION:
                        node.add_subscription(
                            payload.get("topic", ""), 
                            self._in.send,
                            payload.get("message_type", String),
                            payload.get("qos_profile", 10)
                        )
                    elif action == NodeActions.PUBLISH:
                        node.publish(
                            payload.get("topic", ""), 
                            payload.get("message_type", String),
                            payload.get("msg", "")
                        )
                    elif action == NodeActions.UNSUBSCRIBE:
                        node.unsubscribe(payload.get("topic", ""))

                # time.sleep(node.TICK)

            node.destroy_node()
        except Exception as e:
            print(e)

        finally:
            rclpy.shutdown()

    def send_command(self, cmd: NodeAction):
        self._out.send(cmd)

    def get_msg(self) -> RosMessage | None:
        if self._out.poll():
            return self._out.recv()
        
        return None

    def stop(self):
        self._stop_event.set()
        self.join(10)

class Ros2Interface:
    config: Ros2Config
    _process: None | Ros2Process = None
    _thread: None | Thread = None
    _stop_event: ThreadEvent

    def __init__(self, config: Ros2Config, fn: Callable[[RosMessage], None] | None = None):
        self.config = config
        self._process = None
        self._callback = fn
        self._stop_event = ThreadEvent()

    @staticmethod
    def get_blocks():
        return {
            "default_types": [
                ('Bool', 'Bool'),
                ('Byte', 'Byte'),
                ('Char', 'Char'),
                ('Float64', 'Float64'),
                ('Int64', 'Int64'),
                ('UInt64', 'UInt64'),
                ('String', 'String'),
                ('ColorRGBA', 'ColorRGBA')
            ]
        }

    @staticmethod
    def on_message(message: RosMessage):
        print(f"[DEFAULT] Messaggio ricevuto: {message}")

    def wrap_callback(self, fn: Callable[[RosMessage], None]):
        while not self._stop_event.is_set():
            msg = self.get_msg()
            if msg:
                fn(msg)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *attr):
        self.stop()
        
    def start(self):
        if self._process:
            self.stop()

        self._process = Ros2Process()
        self._process.start()

        for p in self.config.publishers:
            self.add_publisher(
                p.topic,
                p.message_type,
                p.qos_profile
            )

        for s in self.config.subscriptions:
            self.add_subscription(
                s.topic,
                s.message_type,
                s.qos_profile
            )

        if self._callback:
            self._thread = Thread(target=self.wrap_callback, args=(self._callback, ), daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()

        if self._process:
            self._process.stop()
        
        if self._thread:
            self._thread.join()

        self._process = None
        self._thread = None

    def add_publisher(self, topic: str, message_type: Any, qos_profile: QoSProfile | int = 10):
        if self._process:
            self._process.send_command(
                NodeAction.AddPubblisher(
                    dict(topic=topic, message_type=message_type, qos_profile=qos_profile)
                )
            )

    def add_subscription(self, topic: str, message_type: Any, qos_profile: QoSProfile | int = 10):
        if self._process:
            self._process.send_command(
                NodeAction.AddSubscription(
                    dict(topic=topic, message_type=message_type, qos_profile=qos_profile)
                )
            )

    def unsubscribe(self, topic: str):
        if self._process:
            self._process.send_command(
                NodeAction.Unsubscribe(
                    dict(topic=topic)
                )
            )

    def publish(self, topic: str, message_type: Any, msg: Any) -> bool:
        if self._process and message_type:
            self._process.send_command(
                NodeAction.Publish(
                    dict(topic=topic, message_type=message_type, msg=msg)
                )
            )
            return True
        
        return False
    
    def get_msg(self) -> RosMessage | None:
        if self._process:
            return self._process.get_msg()
        
        return None