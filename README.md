# SLAM Benchmark Competition

This project facilitates the evaluation of dockerized SLAM systems for the CRL competition. It provides an automated pipeline to run and evaluate SLAM systems using ROS1-based Docker containers.

---

## ⚡ TL;DR - Quick Usage Guide

1. **Ensure prerequisites are installed:** [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/).
2. **Modify the `.env` file** to set dataset paths, SLAM parameters, and the SLAM system to be used.
3. **Run the full SLAM pipeline:**

```bash
./run_pipeline.sh
```

✔ This will:
- Start the SLAM system in a Docker container.
- Play the selected bagfile.
- Stop the SLAM system after playback.
- Evaluate the estimated trajectory.
- Open the evaluation report automatically.

4. **To visualize the SLAM output in RViz**, run the following in your terminal:

```bash
xhost +
```

---

## 📦 Prerequisites

Ensure the following dependencies are installed:

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

---

## 🚀 Setting Up the Dataset

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

### Dataset Structure

Ensure the dataset follows this structure:

```
|-- $BAGFILES_PATH_HOST
    |-- $DATASET_NAME
        |-- $DATASET_NAME.bag  # Bagfile to be evaluated
        |-- reference
            |-- $DATASET_NAME.txt  # Ground truth trajectory
```

---

## 🏃 Running the Full SLAM Pipeline

Execute the entire pipeline in a single command:

```bash
./run_pipeline.sh
```

### What Happens When You Run This?

- The SLAM container starts.
- The bagfile plays according to `SENSOR_MODE`.
- The SLAM container is automatically stopped after playback.
- The estimated trajectory is evaluated.
- The evaluation report (`trajectory_analysis.pdf`) is opened.

To manually start components, use:

```bash
docker compose up run_slam  # Start SLAM
```

```bash
docker compose up play_bag  # Play dataset
```

```bash
docker compose up evaluate_trajectory  # Evaluate trajectory
```

---

## 🛠 Troubleshooting

If you encounter any issues, refer to the common problems and solutions below.

- **Odometry logger is not writing data**
  - Ensure `/estimated_odom` is correctly published by your SLAM system.
- **Sensors not playing as expected**
  - Check `SENSOR_MODE` in the `.env` file. Ensure it is correctly set.
  - If `passive_only` is set, LiDAR topics will not be played.
- **RViz is not displaying SLAM output**
  - Run `xhost +` in your terminal before launching the pipeline.

---

## 📜 License

This project is open-source. Feel free to modify and extend it! 🚀

