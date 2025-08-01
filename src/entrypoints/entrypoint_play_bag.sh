#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Source ROS environment. roscore is guaranteed to be running by docker-compose.
source /opt/ros/noetic/setup.bash

export BAG_PATH="/rosbag_files/$BAGFILE_NAME"
echo "Looking for bag(s) at: $BAG_PATH ($BAGFILES_PATH_HOST/$BAGFILE_NAME on host)"

# This variable will hold the file path(s) for rosbag play
BAG_FILES_TO_PLAY=""

# Check if the path is a file or a directory
if [ -f "$BAG_PATH" ]; then
    echo "Found a single bag file."
    BAG_FILES_TO_PLAY="$BAG_PATH"
    echo "Bag file information:"
    ls -lh "$BAG_FILES_TO_PLAY"
    rosbag info "$BAG_FILES_TO_PLAY"
elif [ -d "$BAG_PATH" ]; then
    echo "Found a directory. Looking for .bag files inside..."
    # Check for .bag files and count them
    BAG_COUNT=$(ls -1q "$BAG_PATH"/*.bag 2>/dev/null | wc -l)
    if [ "$BAG_COUNT" -eq 0 ]; then
        echo "Error: Directory exists, but no .bag files were found inside."
        exit 1
    fi
    echo "Found $BAG_COUNT bag files. They will be played together."
    # Use a glob pattern to play all bag files
    BAG_FILES_TO_PLAY="$BAG_PATH/*.bag"
    echo "Bag files information:"
    python3 scripts/rosbag_info_combined.py $BAG_FILES_TO_PLAY
else
    echo "Error: Path not found (neither a file nor a directory): $BAG_PATH"
    exit 1
fi

# Check if topics file exists
if [ -f "/rosbag_files/$TOPICS_FILE" ]; then
    echo "Using topics file: /rosbag_files/$TOPICS_FILE"
    # topics file is list of topics separated by a new line, we want to make it a list of topics
    TOPICS=$(cat /rosbag_files/$TOPICS_FILE | tr '\n' ' ')
    
else
    echo "No topics file found. Using default topics."
    ls -lh /rosbag_files
fi

# --- NEW SECTION: Wait for roscore to be available ---
echo "Verifying connection to ROS Master at $ROS_MASTER_URI..."
until rostopic list > /dev/null 2>&1; do
  echo "Waiting for roscore to be available..."
  sleep 1
done
echo "Successfully connected to ROS Master."
# ----------------------------------------------------

ROSBAG_PLAY_COMMAND="rosbag play $BAG_FILES_TO_PLAY --clock --quiet -r$ROSBAG_PLAY_RATE"
# if TOPICS is not empty, play only the topics in TOPICS
if [ -n "$TOPICS" ]; then
    echo "Playing only the topics in TOPICS..."
    ROSBAG_PLAY_COMMAND="$ROSBAG_PLAY_COMMAND --topics ${TOPICS}"
fi

echo "Playing the bagfile..."
echo $ROSBAG_PLAY_COMMAND

if ! $ROSBAG_PLAY_COMMAND; then
    echo "Error: Failed to play the bagfile."
    exit 1
fi

echo "Played the bagfile. Exiting with success."
exit 0