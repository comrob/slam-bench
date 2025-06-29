FROM ros:noetic

# Install system dependencies for ROS and Python
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbag \
    ros-noetic-roslaunch \
    ros-noetic-rospy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install specific versions of Python dependencies
RUN pip3 install numpy==1.21.0 pandas==1.3.0 matplotlib==3.4.2

# Install evo package
RUN pip3 install evo --upgrade

# Set up the working directory
WORKDIR /rosbag_player

# Copy scripts into the container
COPY src/scripts/odometry_logger.py /rosbag_player/scripts/odometry_logger.py
COPY src/scripts/trajectory_analysis.py /rosbag_player/scripts/trajectory_analysis.py
COPY src/scripts/rosbag_info_combined.py /rosbag_player/rosbag_info_combined.py
COPY src/entrypoints/entrypoint_bagplay.sh /rosbag_player/entrypoint_bagplay.sh
COPY src/entrypoints/entrypoint_evaluate.sh /rosbag_player/entrypoint_evaluate.sh
COPY src/entrypoints/entrypoint_record_odometry.sh /rosbag_player/entrypoint_record_odometry.sh

# Ensure scripts are executable
RUN chmod +x /rosbag_player/entrypoint_bagplay.sh
RUN chmod +x /rosbag_player/scripts/odometry_logger.py
RUN chmod +x /rosbag_player/entrypoint_evaluate.sh

# Set entrypoint
ENTRYPOINT ["/rosbag_player/entrypoint_bagplay.sh"]