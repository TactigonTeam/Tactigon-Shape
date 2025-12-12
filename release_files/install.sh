#!/bin/bash
set -e

echo "======================================="
echo "  TACTIGON SHAPES — INSTALLER"
echo "======================================="

REQUIRED_OLLAMA="0.12.10"
APP_TAR="tactigon-shapes.tar.gz"

echo ""
echo "🔍 Checking installer files..."
if [ ! -f "$APP_TAR" ]; then
  echo " ERROR: '$APP_TAR' not found!"
  echo "Place the file in the same folder as install.sh"
  exit 1
fi
echo " $APP_TAR found."

echo ""
echo " Creating Docker network (if missing)..."
if ! docker network inspect ollama-net >/dev/null 2>&1; then
    docker network create ollama-net
    echo " Created network: ollama-net"
else
    echo " Network already exists: ollama-net"
fi

echo ""
echo "🔄 Checking local Ollama installation..."
VERSION=$(curl -s http://localhost:11434/api/version | jq -r .version 2>/dev/null || echo "")

if [ -z "$VERSION" ] || [ "$VERSION" = "null" ]; then
    echo " Ollama not running locally. Installing REQUIRED version: $REQUIRED_OLLAMA"
else
    echo " Detected Ollama version: $VERSION"
    if [ "$VERSION" != "$REQUIRED_OLLAMA" ]; then
        echo " Wrong Ollama version! Required: $REQUIRED_OLLAMA"
        echo "Stopping old Ollama container..."
        docker rm -f ollama >/dev/null 2>&1 || true
    fi
fi

echo ""
echo " Starting Ollama container (version $REQUIRED_OLLAMA)..."
docker rm -f ollama >/dev/null 2>&1 || true

docker run -d \
  --name ollama \
  --network ollama-net \
  -p 11434:11434 \
  -v ollama_data:/root/.ollama \
  ollama/ollama:$REQUIRED_OLLAMA

echo "⏳ Waiting for Ollama to initialize..."
sleep 7

echo ""
echo "📥 Checking for Mistral model..."
if docker exec ollama ollama list | grep -q "mistral"; then
    echo "✔ Mistral already installed."
else
    echo "⬇️ Downloading Mistral model..."
    docker exec ollama ollama pull mistral
fi

echo ""
echo " Loading Tactigon Shapes image..."
docker load -i "$APP_TAR"

echo ""
echo "INSTALLATION COMPLETE!"
echo "Run './start.sh' to start both services."
echo "======================================="
