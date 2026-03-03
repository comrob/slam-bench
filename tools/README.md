# Dataset Preparation Tools

This directory contains offline utilities for preparing and processing datasets before SLAM evaluation.

**Requirements:** Python 3.10+, Poetry

## Setup

Install dependencies using Poetry:

```bash
cd tools
poetry install
```

## Available Tools

### extract_gps_trajectory.py

Extracts reference trajectories from MCAP bagfiles containing GPS (NavSatFix) messages.

**Purpose:** Generate ground truth trajectory files in TUM format from GPS data for SLAM benchmarking.

**Features:**
- Reads MCAP files directly (offline processing, no ROS2 runtime needed)
- Converts GPS lat/lon/alt coordinates to local Cartesian frame using UTM projection
- Sets first GPS point as local origin (0, 0, 0)
- Outputs TUM format: `timestamp x y z qx qy qz qw`
- Uses identity quaternion (0, 0, 0, 1) for all poses (GPS doesn't provide orientation)

**Usage:**

```bash
# Install dependencies first
cd tools
poetry install

# Basic usage - extract from default topic
poetry run python extract_gps_trajectory.py \
  --bagfile /path/to/data.mcap \
  --output reference_trajectory.txt

# Or using the installed script
poetry run extract-gps \
  --bagfile /path/to/data.mcap \
  --output reference_trajectory.txt

# Specify custom GPS topic
poetry run python extract_gps_trajectory.py \
  --bagfile /path/to/data.mcap \
  --topic /gnss/custom/fix \
  --output reference.txt
```

**Arguments:**
- `--bagfile`: Path to MCAP file (required)
- `--topic`: GPS topic name (default: `/gnss/septentrio/fix`)
- `--output`: Output trajectory file path (default: `reference_trajectory.txt`)

**Input Format:**
- MCAP file containing `sensor_msgs/msg/NavSatFix` messages
- GPS messages should have valid latitude, longitude, and altitude

**Output Format:**
- TUM format: `timestamp x y z qx qy qz qw`
- Timestamp: Unix time in seconds (with fractional part)
- Position: Meters relative to first GPS point (UTM projection)
- Orientation: Identity quaternion (0, 0, 0, 1)

**Example Output:**
```
1627384952.123456 0.0 0.0 0.0 0.0 0.0 0.0 1.0
1627384952.223456 0.512 -0.103 0.015 0.0 0.0 0.0 1.0
1627384952.323456 1.024 -0.206 0.030 0.0 0.0 0.0 1.0
```

## Notes

- These tools are for **dataset preparation only** (offline processing)
- They are not part of the real-time SLAM evaluation pipeline
- Tools run on the host machine, separate from Docker containers
- Dependencies are managed via Poetry to avoid system-wide installation conflicts
