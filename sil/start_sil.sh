#!/bin/bash
# AEB SIL Environment Startup Script
# Run from WSL2 Ubuntu-22.04, from inside the sil/ directory:
#   cd <repo>/sil && bash ./start_sil.sh

set -e

echo "=== AEB SIL Environment ==="

# Start Docker daemon if not running
if ! pgrep -x dockerd > /dev/null; then
    echo "Starting Docker daemon..."
    sudo dockerd &>/tmp/dockerd.log &
    sleep 8
fi

# Verify Docker is running
sudo docker info > /dev/null 2>&1 || { echo "ERROR: Docker not running"; exit 1; }
echo "Docker: OK"

# Set up vcan0 (optional — TCP CAN is the default transport)
sudo modprobe vcan 2>/dev/null || true
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0 2>/dev/null || true
ip link show vcan0 >/dev/null 2>&1 && echo "vcan0: $(ip link show vcan0 | grep -o 'state [A-Z]*')"

# Operate from the directory holding docker-compose.yml (this script's directory)
cd "$(dirname "$0")"

# Build and run
echo "=== Starting docker compose ==="
sudo docker compose up --build
