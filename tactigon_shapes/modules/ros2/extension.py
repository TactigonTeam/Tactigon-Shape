import rclpy
import time
from rclpy.node import Node, QoSProfile
from std_msgs.msg import String
from threading import Thread, Event
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


class Ros2Thread(Thread):
    def __init__(self):
        Thread.__init__(self, daemon=True)
        self._stop_event = Event()
        self._queue = Queue()

    def run(self):
        rclpy.init()
        node = ShapeNode()

        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0)
            try:
                node_action: NodeAction = self._queue.get_nowait()
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
                        payload.get("fn", print),
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

            except Empty as e:
                pass

        time.sleep(node.TICK)

        node.destroy_node()
        rclpy.shutdown()

    def send_command(self, cmd: NodeAction):
        self._queue.put_nowait(cmd)

    def stop(self):
        self._stop_event.set()

class Ros2Interface:
    config: Ros2Config
    _thread: None | Ros2Thread

    def __init__(self, config: Ros2Config, fn: Callable[[RosMessage], None] | None = None):
        self.config = config
        self._thread = None
        self._callback = fn or self.on_message

    def on_message(self, message: RosMessage):
        print(f"[DEFAULT] Messaggio ricevuto: {message}")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *attr):
        self.stop()
        
    def start(self):
        if self._thread:
            self.stop()

        self._thread = Ros2Thread()
        self._thread.start()

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

    def stop(self):
        if self._thread:
            self._thread.stop()

    def add_publisher(self, topic: str, message_type: Any, qos_profile: QoSProfile | int = 10):
        if self._thread:
            self._thread.send_command(
                NodeAction.AddPubblisher(
                    dict(topic=topic, message_type=message_type, qos_profile=qos_profile)
                )
            )

    def add_subscription(self, topic: str, message_type: Any, qos_profile: QoSProfile | int = 10):
        if self._thread:
            self._thread.send_command(
                NodeAction.AddSubscription(
                    dict(topic=topic, fn=self._callback, message_type=message_type, qos_profile=qos_profile)
                )
            )

    def unsubscribe(self, topic: str):
        if self._thread:
            self._thread.send_command(
                NodeAction.Unsubscribe(
                    dict(topic=topic)
                )
            )

    def publish(self, topic: str, msg: Any) -> bool:
        message_type = next((p.message_type for p in self.config.publishers if p.topic == topic), None)

        if self._thread and message_type:
            self._thread.send_command(
                NodeAction.Publish(
                    dict(topic=topic, message_type=message_type, msg=msg)
                )
            )
            return True
        
        return False