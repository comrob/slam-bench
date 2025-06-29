#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "Starting ROS core..."
roscore &

until rostopic list > /dev/null 2>&1; do
  echo "Waiting for roscore..."
  sleep 1
done

echo "Starting odometry logger..."

python3 scripts/odometry_logger.py