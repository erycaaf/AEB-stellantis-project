#!/bin/bash
# AEB SIL Environment Startup Script
# Run from WSL2 Ubuntu (anywhere — the script resolves its own location):
#   bash <repo>/sil/start_sil.sh

set -e

# Resolve this script's directory (where docker-compose.yml lives).
SIL_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== AEB SIL Environment ==="
echo "SIL directory: ${SIL_DIR}"

# Start Docker daemon if not running
if ! pgrep -x dockerd > /dev/null; then
    echo "Starting Docker daemon..."
    sudo dockerd &>/tmp/dockerd.log &
    sleep 8
fi

# Verify Docker is running
sudo docker info > /dev/null 2>&1 || { echo "ERROR: Docker not running"; exit 1; }
echo "Docker: OK"

# Set up vcan0 (legacy — the SIL uses TCP CAN now, but kept for compatibility
# with older docker-compose.yml variants).
sudo modprobe vcan 2>/dev/null || true
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0 2>/dev/null || true

# Build and run
cd "${SIL_DIR}"
echo "=== Starting docker compose ==="
sudo docker compose up --build
