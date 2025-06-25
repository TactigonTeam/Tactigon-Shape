#!/bin/bash

export DISPLAY=${DISPLAY:-:0}

if ! xhost +local:docker &>/dev/null; then
    echo "Failed to run 'xhost +local:docker'."
    echo "Make sure you're in a graphical session with X11 running."
    exit 1
fi

echo "✅ X11 access granted to Docker. Starting docker-compose..."
docker-compose up --build 