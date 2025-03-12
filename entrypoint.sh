#!/bin/bash
source /opt/ros/noetic/setup.bash

# # Ensure a bag file is provided
# if [ -z "$BAGFILE" ]; then
#     echo "Error: No bag file specified. Set BAGFILE environment variable."
#     exit 1
# fi

echo "Starting ROS core..."
roscore &

# Wait for roscore to start
sleep 5

echo "Starting odometry logger..."
python3 /rosbag_player/scripts/odometry_logger.py &

# Wait a bit to ensure odometry logger starts properly
sleep 2

export BAGFILE="/rosbag_files/$BAGFILE_NAME"
echo "Playing bag file: $BAGFILE ($BAGFILES_PATH_HOST/$BAGFILE_NAME on host)"
rosbag play $BAGFILE --clock
