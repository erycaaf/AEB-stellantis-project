#!/bin/bash
# Quick launcher for AEB simulation
# Usage: ./run.sh [scenario]
# Examples:
#   ./run.sh ccrs_40
#   ./run.sh ccrm
#   ./run.sh ccrb_d6_g40

SCENARIO=${1:-ccrs_40}

echo "=== Killing old processes ==="
killall -9 gzserver gzclient 2>/dev/null
sleep 1

echo "=== Building ==="
cd ~/aeb_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aeb_gazebo
source install/setup.bash

echo "=== Launching scenario: $SCENARIO ==="
ros2 launch aeb_gazebo aeb_with_dashboard.launch.py scenario:=$SCENARIO
