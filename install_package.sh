#!/bin/bash
set -e

echo "======================================="
echo "   Tactigon Shapes (Online Installer)"
echo "======================================="

# 1. Check for the ONLY required local file: your custom app
if [ ! -f "tactigon-shapes.tar" ]; then
  echo "ERROR: tactigon-shapes.tar is missing!"
  exit 1
fi

echo ""
echo "=== Loading Tactigon Shapes Image ==="
docker load -i tactigon-shapes.tar

echo ""
echo "=== Creating Network 'ollama-net' ==="
if ! docker network inspect ollama-net >/dev/null 2>&1; then
  docker network create ollama-net
fi

echo ""
echo "=== Starting Ollama Service ==="
# Stop old container
docker rm -f ollama >/dev/null 2>&1 || true

# Start Ollama (This will download the image from Docker Hub if missing)
# We still keep the volume to save the model after the first download
docker run -d \
  -v ollama_data:/root/.ollama \
  --name ollama \
  -p 11434:11434 \
  --network ollama-net \
  ollama/ollama:latest

echo "Waiting for Ollama to initialize..."
sleep 5

echo ""
echo "=== Checking/Downloading Mistral Model ==="
# Check if mistral is already installed in the container
if docker exec ollama ollama list | grep -q "mistral"; then
    echo "Mistral model already exists. Skipping download."
else
    echo "Mistral model not found. Downloading now..."
    docker exec ollama ollama pull mistral
fi

echo ""
echo "=== Starting Tactigon Shapes App ==="

# X11 Check
if [ -z "$DISPLAY" ]; then
  echo "WARNING: DISPLAY var not set. GUI might fail. try "xhost +""
else
  echo "Granting X11 access..."
  xhost +local:docker >/dev/null 2>&1 || true
fi

docker rm -f tactigon-shapes >/dev/null 2>&1 || true

docker run \
  --name tactigon-shapes \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -p 5123:5123 \
  -v tactigon-config:/app/config \
  -v /var/run/dbus:/var/run/dbus \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e SDL_AUDIODRIVER=dummy \
  --network ollama-net \
  tactigon-shapes

echo ""
echo "======================================="
echo "Installation Complete!"
echo "Tactigon Shapes:    http://localhost:5123"
echo "Ollama:             http://localhost:11434"
echo "======================================="