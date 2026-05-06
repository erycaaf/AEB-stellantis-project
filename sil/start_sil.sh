#!/bin/bash
# AEB SIL Environment Startup Script
# Works on:
#   - Windows + Docker Desktop with WSL2 integration
#   - WSL2 Ubuntu with Docker Engine installed inside the distro
#   - Native Linux with Docker Engine
#
# Run from inside the sil/ directory:
#   cd <repo>/sil && bash ./start_sil.sh

set -e

echo "=== AEB SIL Environment ==="

# Pick `sudo` only when the current user can't talk to the docker socket
# directly. Docker Desktop sets ownership so the WSL user can run `docker`
# without sudo; raw Docker Engine in WSL2 typically requires sudo unless
# the user is in the `docker` group.
DOCKER="docker"
if ! docker info >/dev/null 2>&1; then
    if sudo -n docker info >/dev/null 2>&1 || sudo docker info >/dev/null 2>&1; then
        DOCKER="sudo docker"
    fi
fi

# If Docker still isn't responding, try to start the engine ourselves.
# (Skipped automatically when Docker Desktop is in charge — `docker info`
# already succeeded and we never get here.)
if ! $DOCKER info >/dev/null 2>&1; then
    if command -v dockerd >/dev/null 2>&1; then
        echo "Docker daemon not running — starting dockerd..."
        # Run the redirect inside the sudo'd shell so the log file is
        # opened with root privileges (silences shellcheck SC2024).
        sudo sh -c 'dockerd >/tmp/dockerd.log 2>&1' &
        # Wait up to 15 s for the socket to come up
        for _ in $(seq 1 15); do
            sleep 1
            $DOCKER info >/dev/null 2>&1 && break
        done
    fi
fi

$DOCKER info >/dev/null 2>&1 || {
    echo "ERROR: Docker is not reachable. Either start Docker Desktop or"
    echo "       ensure 'dockerd' is installed inside this WSL distro."
    exit 1
}
echo "Docker: OK ($DOCKER)"

# Operate from the directory holding docker-compose.yml (this script's
# directory) so the script works regardless of where it's invoked from.
cd "$(dirname "$0")"

echo "=== Starting docker compose ==="
$DOCKER compose up --build
