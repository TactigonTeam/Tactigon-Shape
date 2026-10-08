#!/bin/bash
set -e

# Source ROS2 environment
source /opt/vulcanexus/jazzy/setup.bash
source ./install/setup.bash

exec ros2 run example_node example_node /app/example_node/config.json