FROM ros:noetic

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbag \
    ros-noetic-roslaunch \
    ros-noetic-rospy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install numpy==1.21.0 pandas==1.3.0 matplotlib==3.4.2 evo --upgrade

# Set up a standard working directory
WORKDIR /app

# Copy the entire src directory into the WORKDIR
# This preserves the structure (e.g., /app/entrypoints, /app/scripts)
COPY src/ .

# Ensure all entrypoints are executable with a single command
RUN chmod +x entrypoints/*.sh

# Set entrypoint using the new, predictable path
ENTRYPOINT ["/app/entrypoints/entrypoint_bagplay.sh"]