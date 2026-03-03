#!/usr/bin/env python3
import sys
import os
import subprocess

def get_collection_info(collection_dirs):
    """
    Prints ros2 bag info for one or more ROS 2 bag collection directories.
    """
    print(f"Processing {len(collection_dirs)} collection(s)...")
    for collection in collection_dirs:
        print(f"\n--- {collection} ---")
        metadata_file = os.path.join(collection, "metadata.yaml")
        if not os.path.isfile(metadata_file):
            print(f"Error: metadata.yaml missing in {collection}")
            continue

        try:
            subprocess.run(["ros2", "bag", "info", collection], check=True)
        except subprocess.CalledProcessError as error:
            print(f"Error running ros2 bag info on {collection}: {error}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python rosbag_info_combined.py <collection_dir1> <collection_dir2> ...")
        sys.exit(1)

    get_collection_info(sys.argv[1:])