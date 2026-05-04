#!/bin/bash
set -e

# Source ROS2 environment
source /opt/vulcanexus/jazzy/setup.bash
source ./install/setup.bash

echo "Avvio il nodo web_video_server..."
ros2 run web_video_server web_video_server --ros-args -p port:=8081 -p address:=0.0.0.0 &

echo "Avvio il nodo della telecamera ROS 2..."
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p pixel_format:="mjpeg2rgb" \
  -p image_width:=640 \
  -p image_height:=480 \
  -p brightness:=-1 &

echo "Avvio il nodo di tracking..."
exec ros2 run camera_tracking camera_tracking /app/camera_tracking/camera_tracking.json