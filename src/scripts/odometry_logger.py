#!/usr/bin/env python3
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry

# --- Configuration ---
# The directory where the trajectory file will be saved.
# This path should be accessible from within your ROS container/environment.
OUTPUT_PATH_DIR = "/trajectory_files"

# The name of the output file. It can be set via an environment variable.
OUTPUT_FILE_NAME = os.getenv("OUTPUT_FILE_NAME", "estimated_trajectory.txt")
if len(OUTPUT_FILE_NAME) == 0:
    OUTPUT_FILE_NAME = "estimated_trajectory.txt"

# The full path to the output CSV file.
CSV_FILE = os.path.join(OUTPUT_PATH_DIR, OUTPUT_FILE_NAME)


class OdometryLogger(Node):
    def __init__(self):
        super().__init__("odometry_logger")
        self._last_log_time = 0.0

        try:
            os.makedirs(OUTPUT_PATH_DIR, exist_ok=True)
            self.get_logger().info(f"Output directory is set to: {OUTPUT_PATH_DIR}")
            with open(CSV_FILE, "w", encoding="utf-8"):
                pass
            self.get_logger().info(f"Successfully created/cleared trajectory file: {CSV_FILE}")
        except OSError as error:
            self.get_logger().error(f"Failed to create directory or file: {error}")
            raise

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, "/estimated_odom", self.odometry_callback, qos_profile)
        self.get_logger().info("Odometry logger started. Listening to /estimated_odom...")

    def odometry_callback(self, msg: Odometry):
        """
        Callback function to log odometry data in the TUM format.
        This function is called every time a new message is received on the /estimated_odom topic.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        try:
            with open(CSV_FILE, "a", encoding="utf-8") as output_file:
                writer = csv.writer(output_file, delimiter=" ")
                writer.writerow([timestamp, x, y, z, qx, qy, qz, qw])
        except IOError as error:
            self.get_logger().error(f"Could not write to file {CSV_FILE}: {error}")

        now = time.time()
        if now - self._last_log_time > 10.0:
            self.get_logger().info(f"Logged odometry data to {CSV_FILE}")
            self._last_log_time = now

def main():
    """
    Initializes the ROS node, creates the output file, and starts logging.
    """
    rclpy.init()
    node = OdometryLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
