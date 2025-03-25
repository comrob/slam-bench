#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "Starting ROS core..."
roscore &

# Wait for roscore to start
sleep 5

echo "Starting odometry logger..."
python3 /rosbag_player/scripts/odometry_logger.py &
ODOM_LOGGER_PID=$!

# Wait a bit to ensure odometry logger starts properly
sleep 2

export BAGFILE="/rosbag_files/$DATASET_NAME/$BAGFILE_NAME"
echo "Playing bag file: $BAGFILE ($BAGFILES_PATH_HOST/$DATASET_NAME/$BAGFILE_NAME on host)"
# check if the bag file exists
if [ ! -f "$BAGFILE" ]; then
    echo "Bag file not found: $BAGFILE"
    exit 1
else
    echo "Bag file found: $BAGFILE"
    # print full information about the bag file
    ls -lh "$BAGFILE"
    rosbag info "$BAGFILE"
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
if [ "$SENSOR_TRACKS" = "passive_only" ]; then
    echo "Playing only passive sensors..."
    rosbag play $BAGFILE --clock -r$ROSBAG_PLAY_RATE --topics "${PASSIVE_SENSORS[@]}"
else
    echo "Playing all sensors..."
    rosbag play $BAGFILE --clock --quiet -r$ROSBAG_PLAY_RATE --topics "${PASSIVE_SENSORS[@]}" "${ACTIVE_SENSORS[@]}"
fi

if ps -p "$ODOM_LOGGER_PID" > /dev/null 2>&1; then
    echo "Odometry logger is still running. Exiting with success."
    exit 0
else
    echo "Odometry logger is not running, exiting with error."
    exit 1
fi