#!/usr/bin/env python3
import rospy
import csv
import os
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


def odometry_callback(msg):
    """
    Callback function to log odometry data in the TUM format.
    This function is called every time a new message is received on the /estimated_odom topic.
    """
    # Extract position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    # Extract orientation (in TUM format order: qx, qy, qz, qw)
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    
    # Extract timestamp
    timestamp = msg.header.stamp.to_sec()

    # Append the pose to the file
    try:
        with open(CSV_FILE, "a") as f:
            # Use a space as a delimiter for the TUM format
            writer = csv.writer(f, delimiter=" ")
            # Write the data row: timestamp tx ty tz qx qy qz qw
            writer.writerow([timestamp, x, y, z, qx, qy, qz, qw])
    except IOError as e:
        rospy.logerr_throttle(1.0, f"Could not write to file {CSV_FILE}: {e}")
    
    # Log the received data to the console (throttled to once every 10 seconds)
    rospy.loginfo_throttle(10.0, f"Logged odometry data to {CSV_FILE}")


def main():
    """
    Initializes the ROS node, creates the output file, and starts logging.
    """
    rospy.init_node("odometry_logger", anonymous=True)

    # --- Setup Directory and File ---
    # This section runs once when the node starts.
    try:
        # Ensure the output directory exists.
        os.makedirs(OUTPUT_PATH_DIR, exist_ok=True)
        rospy.loginfo(f"Output directory is set to: {OUTPUT_PATH_DIR}")
        
        # Create a new, empty file (or clear an existing one).
        # This ensures the node starts with a fresh log file every time.
        with open(CSV_FILE, "w") as f:
            pass # This will create an empty file or truncate an existing one.
        rospy.loginfo(f"Successfully created/cleared trajectory file: {CSV_FILE}")

    except OSError as e:
        rospy.logerr(f"Failed to create directory or file: {e}")
        # If we can't create the file/dir, there's no point in continuing.
        return

    # --- Subscribe to Topic ---
    # Subscribe to the /estimated_odom topic.
    # The 'odometry_callback' function will be executed for each message.
    rospy.Subscriber("/estimated_odom", Odometry, odometry_callback)
    
    rospy.loginfo("Odometry logger started. Listening to /estimated_odom...")
    
    # Keep the node running until it's shut down (e.g., by Ctrl+C).
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
