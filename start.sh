#!/bin/bash
set -e

echo "======================================"
echo "  TACTIGON SHAPES — START             "
echo "======================================"

cat .xprofile >> ~/.xprofile
docker compose up --build
