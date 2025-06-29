#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Source the ROS setup script to configure the environment
source "/opt/ros/noetic/setup.bash"

# Execute the command passed to this script (e.g., "roscore" or "roslaunch")
# The 'exec' command replaces the shell process with the given command,
# which is important for proper signal handling (like Ctrl+C).
exec "$@"