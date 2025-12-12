#!/bin/bash
set -e

echo "======================================="
echo "  TACTIGON SHAPES — STARTER (CLEAN)"
echo "======================================="

export DISPLAY=${DISPLAY:-:0}

echo ""
echo " Granting X11 access..."
if ! xhost +local:docker &>/dev/null; then
    echo " WARNING: Could not grant X11 access. GUI may not work."
else
    echo "✔ X11 access granted."
fi

echo ""
echo " Cleaning old containers..."

if docker ps -a | grep -q "tactigon-shapes"; then
    echo "Stopping and removing old tactigon-shapes..."
    docker rm -f tactigon-shapes >/dev/null 2>&1 || true
fi

if docker ps -a | grep -q "ollama"; then
    echo "Stopping and removing old Ollama container..."
    docker rm -f ollama >/dev/null 2>&1 || true
fi

echo "✔ Clean containers removed."

echo ""
echo " Starting fresh Ollama container..."
docker run -d \
  --name ollama \
  --network ollama-net \
  -p 11434:11434 \
  -v ollama_data:/root/.ollama \
  ollama/ollama:0.12.10

echo "⏳ Waiting for Ollama to initialize..."
sleep 5

echo ""
echo " Ensuring Mistral model exists..."
if docker exec ollama ollama list | grep -q "mistral"; then
    echo "✔ Mistral model already installed."
else
    echo " Pulling Mistral model..."
    docker exec ollama ollama pull mistral
fi

echo ""
echo " Starting Tactigon Shapes container..."
docker run -it \
  --name tactigon-shapes \
  --privileged \
  --network ollama-net \
  -p 5123:5123 \
  -e DISPLAY=$DISPLAY \
  -v tactigon-config:/app/config \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /var/run/dbus:/var/run/dbus \
  -e SDL_AUDIODRIVER=dummy \
  tactigon-shapes
  # -v $(pwd):/app \

echo ""
echo "======================================="
echo "Tactigon Shapes running at: http://localhost:5123"
echo "Ollama running at:          http://localhost:11434"
echo "======================================="
