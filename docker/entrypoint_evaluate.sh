#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "Starting trajectory analysis..."
# if TEST_MODE is 1, set --test flag

COMMAND="python3 /rosbag_player/scripts/trajectory_analysis.py"
if [ "$TEST_MODE" == "1" ]; then
    COMMAND="$COMMAND --test"
fi

echo "Running command: $COMMAND"
$COMMAND