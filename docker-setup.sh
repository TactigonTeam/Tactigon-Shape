#!/bin/bash

export DISPLAY=${DISPLAY:-:0}

if ! xhost +local:docker &>/dev/null; then
    echo "Failed to run 'xhost +local:docker'."
    echo "Make sure you're in a graphical session with X11 running."
    exit 1
fi

echo "✅ X11 access granted to Docker"

echo "Building docker image..."
sudo docker build -t tactigon-shapes .

echo "Creating an empty folder to avoid sync venv folder"
mkdir -p /tmp/empty

echo "Starting docker container..."
sudo docker run -it \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -p 5123:5123 \
  -v $(pwd):/app \
  -v /tmp/empty:/app/venv \
  -v /var/run/dbus:/var/run/dbus \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  tactigon-shapes