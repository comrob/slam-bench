#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Source ROS environment. roscore is guaranteed to be running by docker-compose.
source /opt/ros/noetic/setup.bash

echo "Starting odometry logger..."

# The main command is all that's left.
python3 scripts/odometry_logger.py