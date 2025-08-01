#!/bin/bash

# Check if an image name was provided as an argument.
if [ -z "$1" ]; then
  echo "Usage: $0 <image_name:tag>"
  exit 1
fi

# Assign the first argument to the IMAGE_NAME variable.
IMAGE_NAME=$1

# Create a valid filename by replacing colons and slashes with underscores.
OUTPUT_FILENAME=$(echo "$IMAGE_NAME" | tr ':/' '_').tar

echo "Saving Docker image '$IMAGE_NAME' to '$OUTPUT_FILENAME'..."

# Use 'docker save' to create the .tar archive.
# The -o flag specifies the output file.
docker save -o "$OUTPUT_FILENAME" "$IMAGE_NAME"

# Check if the command was successful.
if [ $? -eq 0 ]; then
  echo "✅ Successfully saved image to '$OUTPUT_FILENAME'"
else
  echo "❌ Failed to save image '$IMAGE_NAME'"
  exit 1
fi