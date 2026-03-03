#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Source ROS 2 environment.
source /opt/ros/jazzy/setup.bash

BAG_REL_PATH="${BAGFILE_NAME:-sensors}"
BAG_SEARCH_ROOT="/rosbag_files"
BAG_PATH="${BAG_SEARCH_ROOT}/${BAG_REL_PATH}"
TOPICS=""
PLAY_TARGET=""
PLAY_MODE=""

echo "Looking for MCAP collection at: $BAG_PATH ($BAGFILES_PATH_HOST/$BAG_REL_PATH on host)"

find_mcap_collection() {
    local input_path="$1"

    if [ -f "$input_path/metadata.yaml" ]; then
        echo "$input_path"
        return 0
    fi

    local candidates=()
    while IFS= read -r metadata_file; do
        candidates+=("$(dirname "$metadata_file")")
    done < <(find "$input_path" -mindepth 2 -maxdepth 4 -type f -name metadata.yaml 2>/dev/null | sort)

    if [ "${#candidates[@]}" -eq 1 ]; then
        echo "${candidates[0]}"
        return 0
    fi

    if [ "${#candidates[@]}" -gt 1 ]; then
        echo "Error: Multiple MCAP collections found under $input_path. Please set BAGFILE_NAME explicitly to one collection path:" >&2
        printf '  - %s\n' "${candidates[@]}" >&2
        return 1
    fi

    echo "Error: No metadata.yaml found under $input_path." >&2
    return 1
}

if [ -f "$BAG_PATH" ]; then
    if [[ "$BAG_PATH" != *.mcap ]]; then
        echo "Error: File mode supports only .mcap files: $BAG_PATH"
        exit 1
    fi
    PLAY_TARGET="$BAG_PATH"
    PLAY_MODE="single-file"
    echo "Using direct MCAP file (no metadata.yaml required): $PLAY_TARGET"
    echo "Bag metadata (best effort):"
    ros2 bag info "$PLAY_TARGET" --storage mcap || true
elif [ -d "$BAG_PATH" ]; then
    COLLECTION_DIR="$(find_mcap_collection "$BAG_PATH")"
    METADATA_FILE="$COLLECTION_DIR/metadata.yaml"

    echo "Using MCAP collection: $COLLECTION_DIR"

    if ! grep -Eq '^\s*storage_identifier:\s*mcap\s*$' "$METADATA_FILE"; then
        echo "Error: metadata.yaml does not declare 'storage_identifier: mcap'."
        exit 1
    fi

    PLAY_TARGET="$COLLECTION_DIR"
    PLAY_MODE="collection"
    echo "Bag metadata:"
    ros2 bag info "$PLAY_TARGET"
else
    echo "Error: Path not found: $BAG_PATH"
    exit 1
fi

# Check if topics file exists
if [ -n "${TOPICS_FILE:-}" ] && [ -f "/rosbag_files/$TOPICS_FILE" ]; then
    echo "Using topics file: /rosbag_files/$TOPICS_FILE"
    TOPICS=$(grep -v '^\s*#' "/rosbag_files/$TOPICS_FILE" | xargs)
else
    echo "No topics file found. Using all topics from the bag."
fi

ROSBAG_PLAY_COMMAND=(ros2 bag play "$PLAY_TARGET" --clock --storage mcap --rate "${ROSBAG_PLAY_RATE:-1.0}")

# if TOPICS is not empty, play only the topics in TOPICS
if [ -n "$TOPICS" ]; then
    echo "Playing only the topics listed in TOPICS_FILE..."
    read -r -a TOPICS_ARR <<< "$TOPICS"
    ROSBAG_PLAY_COMMAND+=(--topics "${TOPICS_ARR[@]}")
fi

echo "Playing the bagfile..."
echo "Playback mode: $PLAY_MODE"
printf '%q ' "${ROSBAG_PLAY_COMMAND[@]}"
echo

if ! "${ROSBAG_PLAY_COMMAND[@]}"; then
    echo "Error: Failed to play the bagfile."
    exit 1
fi

echo "Played the bagfile. Exiting with success."
exit 0