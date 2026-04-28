#!/bin/bash
set -e

# Source ROS2 environment
source /opt/vulcanexus/jazzy/setup.bash
source ./install/setup.bash

echo "Avvio il nodo della telecamera ROS 2 (v4l2_camera)..."
# nodo della telecamera
# exec v4l2-ctl --stream-mmap
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p pixel_format:="mjpeg2rgb" \
  -p image_width:=640 \
  -p image_height:=480 \
  -p brightness:=-1 &
# exec ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0" &
# exec ros2 run v4l2_camera v4l2_camera_node --ros-args \
#   -p video_device:="/dev/video0" \
#   -p pixel_format:="MJPG" \
#   -p image_size:="[640,480]" &

# pausa per inizializzare la telecamera
sleep 2 

echo "Avvio il nodo di tracking..."

# nodo principale
exec ros2 run camera_tracking camera_tracking /app/camera_tracking/camera_tracking.json