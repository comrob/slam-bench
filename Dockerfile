FROM ros:jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rclpy \
    ros-jazzy-nav-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed numpy pandas matplotlib evo

# Set up a standard working directory
WORKDIR /app

# Copy the entire src directory into the WORKDIR
# This preserves the structure (e.g., /app/entrypoints, /app/scripts)
COPY src/ .

# Ensure all entrypoints are executable with a single command
RUN chmod +x entrypoints/*.sh

# Set our new script as the default entrypoint for the image
ENTRYPOINT ["/app/entrypoints/ros_entrypoint.sh"]