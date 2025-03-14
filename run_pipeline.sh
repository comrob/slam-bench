#!/bin/bash

# Load environment variables from .env file
if [ -f .env ]; then
    set -o allexport
    source .env
    set +o allexport
fi

# Start the SLAM container in the background
echo "Starting SLAM system..."
docker compose up run_slam &
SLAM_PID=$!

# Wait a few seconds to ensure SLAM starts properly
sleep 5

# Start playing the bagfile
echo "Playing bagfile..."
docker compose up play_bag

# After bag playback finishes, stop the SLAM container
echo "Stopping SLAM system..."
docker compose down run_slam

# Run trajectory evaluation
echo "Running trajectory evaluation..."
docker compose up evaluate_trajectory

# Open the evaluation report
echo "Opening evaluation report..."
REPORT_PATH="$BAGFILES_PATH_HOST/$DATASET_NAME/evaluation_output/trajectory_analysis.pdf"
if [ -f "$REPORT_PATH" ]; then
    xdg-open "$REPORT_PATH" 2>/dev/null || open "$REPORT_PATH" 2>/dev/null || echo "Report available at: $REPORT_PATH"
else
    echo "Evaluation report not found at: $REPORT_PATH"
fi

exit 0