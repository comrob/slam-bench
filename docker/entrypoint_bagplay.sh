#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "Starting ROS core..."
roscore &

until rostopic list > /dev/null 2>&1; do
  echo "Waiting for roscore..."
  sleep 1
done

echo "Starting odometry logger..."
python3 /rosbag_player/scripts/odometry_logger.py &
ODOM_LOGGER_PID=$!

# Wait a bit to ensure odometry logger starts properly
sleep 2

export BAG_PATH="/rosbag_files/$DATASET_NAME/$BAGFILE_NAME"
echo "Looking for bag(s) at: $BAG_PATH ($BAGFILES_PATH_HOST/$DATASET_NAME/$BAGFILE_NAME on host)"

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
    # Loop through and display info for each bag file
    # for bag in $BAG_FILES_TO_PLAY; do
    #   echo "--- Info for $(basename "$bag") ---"
    #   ls -lh "$bag"
    # #   rosbag info "$bag"
    #   echo "---------------------"
    # done
    python3 /rosbag_player/rosbag_info_combined.py $BAG_FILES_TO_PLAY
else
    echo "Error: Path not found (neither a file nor a directory): $BAG_PATH"
    exit 1
fi


# Define sensor topic groups
export PASSIVE_SENSORS=(
    /camera_front/camera_info
    /camera_front/image_raw/compressed
    /flir_boson/camera_info
    /flir_boson/image_raw/compressed
    /flir_boson/image_raw16/compressed
    /plantpix1/ndvi/compressed
    /plantpix1/nir/compressed
    /plantpix1/rgb/compressed
    /plantpix2/ndvi/compressed
    /plantpix2/nir/compressed
    /plantpix2/rgb/compressed
    /imu/data
)

export ACTIVE_SENSORS=(
    /os1/imu
    /os1/metadata
    /os1/points
)

# Construct rosbag play command
BAG_FILES_TO_PLAY="$BAG_FILES_TO_PLAY"
# NOTE: $BAG_FILES_TO_PLAY is intentionally not quoted to allow shell expansion of the wildcard (*)
if [ "$SENSOR_TRACKS" = "passive" ]; then
    echo "Playing only passive sensors..."
    rosbag play $BAG_FILES_TO_PLAY --clock -r$ROSBAG_PLAY_RATE --topics "${PASSIVE_SENSORS[@]}"
else
    echo "Playing all sensors..."
    rosbag play $BAG_FILES_TO_PLAY --clock --quiet -r$ROSBAG_PLAY_RATE
fi


if ps -p "$ODOM_LOGGER_PID" > /dev/null 2>&1; then
    echo "Odometry logger is still running. Exiting with success."
    exit 0
else
    echo "Odometry logger is not running, exiting with error."
    exit 1
fi