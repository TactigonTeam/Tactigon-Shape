import time
import json
from flask import Flask
from threading import Thread, Event
from queue import Queue
from typing import Optional, Tuple

from paho.mqtt.client import Client

from .models import MQTTConfig

class MQTTClient(Thread):
    config: MQTTConfig
    userdata: dict
    pubblish_queue: Queue

    client: Optional[Client] = None

    def __init__(self, config: MQTTConfig):
        Thread.__init__(self)
        self.config = config

    @property
    def connected(self):
        return self.client.is_connected() if self.client else False

    def on_connect(self):
        pass

    def on_disconnect(self, client: Client, *args):
        if not self.client:
            return
        
        reconnect_count = reconnect_delay = 0
        while reconnect_count < self.config.max_reconnect_count:
            time.sleep(reconnect_delay)

            try:
                self.client.reconnect()
                return
            except Exception as err:
                pass

            reconnect_delay *= self.config.reconnect_rate
            reconnect_count += 1

        self.client = None

    def on_message(self, client: Client, userdata: dict, msg):
        print(msg)
        pass

    def run(self):
        self.userdata = {}
        self.client = Client(
            client_id=self.config.client_id,
            callback_api_version=self.config.callback_api_version,
            protocol=self.config.protocol_version,
            userdata=self.userdata,
        )
        # self.client.on_connect=self.on_connect
        self.client.on_disconnect=self.on_disconnect
        self.client.on_message=self.on_message

    def subscribe(self, topic: str, qos: int = 0):
        if not self.client:
            return
        
        self.client.subscribe(
            topic=topic,
            qos=qos
        )

    def pubblish(self, topic: str, payload: dict, qos: int = 0) -> bool:
        if not self.client:
            return False
        
        try:
            self.client.publish(
                topic=topic,
                payload=json.dumps(payload),
                qos=qos,
            )
        except Exception as e:
            return False

        return True
        


class MQTTExtension:
    _thread: Optional[MQTTClient] = None

    def __init__(self, app: Optional[Flask] = None):    
        if app:
            self.init_app(app)

    def init_app(self, app: Flask):
        app.extensions[MQTTExtension.__name__] = self
    
    @property
    def connected(self) -> bool:
        return bool(self._thread.connected) if self._thread else False

    def start(self, config: MQTTConfig):
        if self._thread:
            self.stop()

        self._thread = MQTTClient(config)
        self._thread.start()

    def stop(self):
        if self._thread:
            self._thread.stop()
            self._thread = None

    def pubblish(self, topic: str, payload: dict) -> bool:
        if self._thread:
            self._thread.pubblish(topic, payload)
            return True
        
        return False