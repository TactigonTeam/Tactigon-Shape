#!/bin/bash
set -e

echo "======================================"
echo "  TACTIGON SHAPES — START             "
echo "======================================"
XHOST_CMD="xhost +local:docker"

touch ~/.xprofile
if ! grep -qxF "$XHOST_CMD" ~/.xprofile; then
  # Aggiunge un newline solo se il file non è vuoto e non finisce già con un newline
  [ -s ~/.xprofile ] && [ "$(tail -c1 ~/.xprofile | wc -l)" -eq 0 ] && echo "" >> ~/.xprofile
  # scrive il permesso
  echo "$XHOST_CMD" >> ~/.xprofile
fi
docker compose up --build
