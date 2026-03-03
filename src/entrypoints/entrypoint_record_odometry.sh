#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Source ROS 2 environment.
source /opt/ros/jazzy/setup.bash

echo "Starting odometry logger..."

# The main command is all that's left.
python3 scripts/odometry_logger.py