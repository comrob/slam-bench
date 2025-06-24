#!/usr/bin/env python
import rosbag
import sys
import os
from datetime import datetime

def get_combined_info(bag_files):
    """
    Generates a combined summary for a list of rosbag files.
    """
    total_size_gb = 0
    total_messages = 0
    all_topics = {}
    all_types = set()
    min_start_time = float('inf')
    max_end_time = float('-inf')

    print("Processing {} bag file(s)...".format(len(bag_files)))

    for bag_file in bag_files:
        print("  - {}".format(os.path.basename(bag_file)))
        try:
            bag = rosbag.Bag(bag_file, 'r')
            total_size_gb += os.path.getsize(bag_file) / (1024.0**3)
            
            # Get start and end times
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            if start_time < min_start_time:
                min_start_time = start_time
            if end_time > max_end_time:
                max_end_time = end_time

            # Get topic and type info
            topics_info = bag.get_type_and_topic_info().topics
            total_messages += bag.get_message_count()

            for topic, topic_info in topics_info.items():
                all_types.add(topic_info.msg_type)
                if topic not in all_topics:
                    all_topics[topic] = {
                        'msg_type': topic_info.msg_type,
                        'count': 0,
                        'connections': topic_info.connections
                    }
                all_topics[topic]['count'] += topic_info.message_count

            bag.close()
        except Exception as e:
            print("    Error processing {}: {}".format(bag_file, e))
            continue
    
    # --- Print Combined Summary ---
    duration = max_end_time - min_start_time
    
    print("\n--- Combined Information ---")
    print("path(s):         {}".format(', '.join([os.path.basename(f) for f in bag_files])))
    print("duration:        {:.2f}s".format(duration))
    print("start:           {} ({:.2f})".format(datetime.fromtimestamp(min_start_time).strftime('%b %d %Y %H:%M:%S.%f')[:-3], min_start_time))
    print("end:             {} ({:.2f})".format(datetime.fromtimestamp(max_end_time).strftime('%b %d %Y %H:%M:%S.%f')[:-3], max_end_time))
    print("size:            {:.2f} GB".format(total_size_gb))
    print("messages:        {}".format(total_messages))
    
    print("\ntypes:")
    for msg_type in sorted(list(all_types)):
        print("    {}".format(msg_type))

    print("\ntopics:")
    # Sort topics alphabetically
    for topic in sorted(all_topics.keys()):
        info = all_topics[topic]
        count = info['count']
        freq_str = ""
        if duration > 0:
            frequency = count / duration
            freq_str = " ({} Hz)".format(round(frequency, 2))

        print("    {:<40} {:>10} msgs : {} {}".format(topic, count, info['msg_type'], freq_str))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python rosbag_info_combined.py <file1.bag> <file2.bag> ...")
        print("       or")
        print("       python rosbag_info_combined.py *.bag")
        sys.exit(1)
    
    # The shell expands *.bag, so sys.argv will contain all matching file names
    bag_files = sys.argv[1:]
    get_combined_info(bag_files)