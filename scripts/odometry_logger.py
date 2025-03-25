#!/usr/bin/env python3
import rospy
import csv
import os
from nav_msgs.msg import Odometry

# Ensure that the data directory exists inside the mounted bagfiles folder
OUTPUT_PATH_DIR = "/trajectory_files"
OUTPUT_FILE_NAME = os.getenv("OUTPUT_FILE_NAME", "estimated_trajectory.txt")
if len(OUTPUT_FILE_NAME) == 0:
    OUTPUT_FILE_NAME = "estimated_trajectory.txt"
CSV_FILE = os.path.join(OUTPUT_PATH_DIR, OUTPUT_FILE_NAME)

# os.makedirs(DATA_DIR, exist_ok=True)  # Creates the folder if it doesn't exist

# Write the header if the file does not exist
if not os.path.isfile(CSV_FILE):
    with open(CSV_FILE, "w") as f:
        writer = csv.writer(f)

def odometry_callback(msg):
    """Callback function to log odometry data."""
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    qw = msg.pose.pose.orientation.w
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    timestamp = msg.header.stamp.to_sec()

    # Append the pose to the CSV file
    with open(CSV_FILE, "a") as f:
        writer = csv.writer(f, delimiter=" ")
        # write in TUM format
        writer.writerow([timestamp, x, y, z, qw, qx, qy, qz])
    
    rospy.loginfo_throttle(0.1, f"Logged Odom: timestamp={timestamp:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                            f"q=[{qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]")

def main():
    """Initializes the ROS node and starts logging odometry data."""
    rospy.init_node("odometry_logger", anonymous=True)
    # Clean the file
    with open(CSV_FILE, "w") as f:
        writer = csv.writer(f)
    
    # Subscribe to /estimated_odom
    rospy.Subscriber("/estimated_odom", Odometry, odometry_callback)
    
    rospy.loginfo("Odometry logger started. Listening to /estimated_odom...")
    
    rospy.spin()  # Keep the node running

if __name__ == "__main__":
    main()