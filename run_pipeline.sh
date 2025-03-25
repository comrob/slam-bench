#!/bin/bash

# Load environment variables from .env file
if [ -f .env ]; then
    set -o allexport
    source .env
    set +o allexport
fi

# Determine docker-compose command based on DEV_DOCKER
DOCKER_COMPOSE_CMD="docker compose"
if [ "$DEV_DOCKER" == "true" ]; then
    DOCKER_COMPOSE_CMD="docker compose -f docker-compose.yaml -f docker-compose-dev.yaml"
fi

# Start the SLAM container in the background
echo "Starting SLAM system..."
$DOCKER_COMPOSE_CMD up run_slam &
SLAM_PID=$!

# Wait a few seconds to ensure SLAM starts properly
sleep 5

# Start playing the bagfile
echo "Playing bagfile..."
EVALUATION_OUTPUT_PATH="$BAGFILES_PATH_HOST/$DATASET_NAME/evaluation_output"
mkdir -p "$EVALUATION_OUTPUT_PATH"
$DOCKER_COMPOSE_CMD up play_bag

# After bag playback finishes, stop the SLAM container
echo "Stopping SLAM system..."
$DOCKER_COMPOSE_CMD down run_slam

# Run trajectory evaluation
echo "Running trajectory evaluation..."
$DOCKER_COMPOSE_CMD up evaluate_trajectory

# Open the evaluation report
echo "Opening evaluation report..."
REPORT_PATH="$BAGFILES_PATH_HOST/$DATASET_NAME/evaluation_output/trajectory_analysis.pdf"
if [ -f "$REPORT_PATH" ]; then
    xdg-open "$REPORT_PATH" 2>/dev/null || open "$REPORT_PATH" 2>/dev/null || echo "Report available at: $REPORT_PATH"
else
    echo "Evaluation report not found at: $REPORT_PATH"
fi

exit 0
