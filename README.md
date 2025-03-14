# SLAM Benchmark Competition

This project facilitates the evaluation of dockerized SLAM systems for the CRL competition. It provides three Docker services:

- **`play_bag`**: Plays a bagfile and logs the estimated odometry to a CSV file.
- **`evaluate_trajectory`**: Compares the estimated trajectory against the ground truth trajectory.
- **`run_slam`**: Runs the SLAM system's Docker container.

---

## 📦 Prerequisites

Ensure the following dependencies are installed:

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

---

## 🚀 Quick Start

### 1️⃣ Specify the Bagfiles

This section outlines how to configure the dataset and sensor selection for the SLAM evaluation.

Modify the `.env` file to define:

- **`BAGFILES_PATH_HOST`**: Path to the directory containing the bagfile dataset on the host machine.
- **`DATASET_NAME`**: Name of the dataset folder inside `BAGFILES_PATH_HOST`.
- **`SENSOR_MODE`**: Controls which sensors are played from the bagfile. Options:
  - `passive_only`: Plays only passive sensors (e.g., cameras, IMUs, plant sensors).
  - Any other value (or unset): Plays all sensors, including LiDAR.

To simplify access, you can create a symbolic link for easier dataset management:

```bash
ln -s <your_bagfiles_path> $HOME/bagfiles_competition
```

#### Dataset Structure

Ensure the dataset follows this structure:

```
|-- $BAGFILES_PATH_HOST
    |-- $DATASET_NAME
        |-- $DATASET_NAME.bag  # Bagfile to be evaluated
        |-- reference
            |-- $DATASET_NAME.csv  # Ground truth trajectory
```

---

### 2️⃣ Run Your SLAM System

To execute the example SLAM system:

```bash
docker compose up run_slam
```

- Your SLAM system must run as a ROS1 node and publish estimated odometry to `/estimated_odom`.
- Ensure your SLAM system is dockerized by specifying its Docker image in the `.env` file. You can find our containerized SLAM system at [liorf-crl](https://github.com/comrob/liorf-crl):

```bash
SLAM_IMAGE=ghcr.io/comrob/liorf-crl:latest
```

Alternatively, you can run your SLAM system directly on the host machine without Docker for testing purposes. Ensure that it correctly publishes odometry to `/estimated_odom`.

---

### 3️⃣ Run Dataset Evaluation

To run the dataset and log estimated odometry:

```bash
docker compose up play_bag
```

### What Happens When You Run This?

When you execute the `play_bag` service, it will:

- Start `roscore` (unless already running by the SLAM system).
- Launch the **odometry logger** before playing the bagfile.
- Log `/estimated_odom` data to `$BAGFILES_PATH_HOST/evaluation_output/estimated_trajectory.csv`.
- Play only the specified sensor set based on `SENSOR_MODE`.

---

### 4️⃣ Verify Logged Data

After stopping the container, inspect the logged trajectory:

```bash
cat $BAGFILES_PATH_HOST/evaluation_output/estimated_trajectory.csv
```

📌 **Expected Format (TUM format, no header):**

```
timestamp x y z qw qx qy qz
```

---

### 5️⃣ Evaluate the Trajectory

### Adjusting the Test Mode

The appropriate mode should be selected based on the trajectory length. Smaller datasets require different delta distances for RPE calculation.

```bash
TEST_MODE=1  # For datasets under 50m
TEST_MODE=0  # For datasets over 50m
```

This setting adjusts the RPE calculation method.

To evaluate the trajectory:

```bash
docker compose up evaluate_trajectory
```

Output is saved in `$BAGFILES_PATH_HOST/$DATASET_NAME/evaluation_output/`, containing:

- `estimated_trajectory.csv`: Logged estimated trajectory.
- `trajectory_analysis.pdf`: Trajectory plots and evaluation metrics.
- `trajectory_analysis.yaml`: YAML file with trajectory evaluation metrics.

---

## 🛠 Troubleshooting

If you encounter any issues, refer to the common problems and solutions below.

- **Odometry logger is not writing data**
  - Ensure `/estimated_odom` is correctly published by your SLAM system.
- **Sensors not playing as expected**
  - Check `SENSOR_MODE` in the `.env` file. Ensure it is correctly set.
  - If `passive_only` is set, LiDAR topics will not be played.

---

## 📜 License

This project is open-source. Feel free to modify and extend it! 🚀

