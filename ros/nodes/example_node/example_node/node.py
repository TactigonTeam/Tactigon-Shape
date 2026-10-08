#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Contributors:
# Massimiliano Bellino
# Stefano Barbareschi
#********************************************************************************

import json
from rclpy.node import Node
from std_msgs.msg import String

from example_node.models import ExampleConfig

class ExampleNode(Node):
    config: ExampleConfig

    def __init__(self, config_path: str):
        Node.__init__(self, ExampleNode.__name__)
        self.get_logger().info("Starting Example Node...")

        self.config = self.load_config(config_path)

        self._subscriber = self.create_subscription(String, self.config.subscription_topic, self.on_message, 10)
        self._publisher = self.create_publisher(String, self.config.publish_topic, 10)

        self.get_logger().info(f"Camera Example Node started. Listening on topic: {self.config.subscription_topic}")

    def load_config(self, config_path: str) -> ExampleConfig:
        with open(config_path) as cf:
            return ExampleConfig.FromJSON(json.load(cf))
        
    def on_message(self, msg: String):
        self.get_logger().info(f"Received message: {msg.data}")
        self._publisher.publish(String(data=f"Echo: {msg.data}"))
