#!/bin/bash
set -e

echo "======================================"
echo "  TACTIGON SHAPES — START             "
echo "======================================"

xhost +local:docker
docker compose up -d