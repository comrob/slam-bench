# SLAM Benchmark Competition

This project facilitates the evaluation of dockerized SLAM systems for the CRL competition. It provides an automated pipeline to run and evaluate SLAM systems using ROS1-based Docker containers.

---

## ⚡ TL;DR - Quick Usage Guide

1. **Ensure prerequisites are installed:** [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/).
2. **Download the datasets** (See Setting Up the Dataset Section)
3. **Modify the `.env` file** to set dataset paths, rosbag playing parameters, and the SLAM system to be used.
4. **Run the full SLAM pipeline:**

```bash
./run_pipeline.sh
```

✔ This will:
- Start the SLAM system in a Docker container (liorf-crl by default).
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

## Selecting your SLAM system

In the `.env` file, specify the variable:
```
SLAM_IMAGE=your-slam-image
```
If not specified, the default liorf-crl would be used.

## 🚀 Setting Up the Dataset

You may download the following training data:
- [All](https://comrob-ds.fel.cvut.cz:9001/api/v1/buckets/cb-slam/objects/download?prefix=data/train/) training datasets.
- Separate datasets:
  - [shellby-0225-train-lab](https://comrob-ds.fel.cvut.cz:9001/api/v1/buckets/cb-slam/objects/download?prefix=data/train/shellby-0225-train-lab/) ( 5.5 GB, about 3.3 GB for download)
  - [shellby-0225-train-loop1](https://comrob-ds.fel.cvut.cz:9001/api/v1/buckets/cb-slam/objects/download?prefix=data/train/shellby-0225-train-loop1/) ( 47 GB, about 24 GB for download)
  - [extrinsics](https://comrob-ds.fel.cvut.cz:9001/api/v1/buckets/cb-slam/objects/download?prefix=data/train/extrinsics/) (16 kB)


Modify the `.env` file to define:

- **`BAGFILES_PATH_HOST`**: Path to the directory containing the bagfile dataset on the host machine.
- **`DATASET_NAME`**: Name of the dataset folder inside `BAGFILES_PATH_HOST`.
- **`SENSOR_MODE`**: Controls which sensors are played from the bagfile. Options:
  - `passive`: Plays only passive sensors (e.g., cameras, IMUs, plant sensors).
  - Any other value (or unset): Plays all sensors, including LiDAR.

To simplify access, you can create a symbolic link for easier dataset management:
```bash
ln -s <your_bagfiles_path> $HOME/bagfiles_competition
```

The default configuration assumes that you have downloaded and unpacked [shellby-0225-train-lab](https://comrob-ds.fel.cvut.cz:9001/api/v1/buckets/cb-slam/objects/download?prefix=data/train/shellby-0225-train-lab/) to the `~/bagfiles_competition` folder



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

## Submitting your SLAM solution.

After testing your docker image, we encourage you to submit to the [slam benchmark competition](https://comrob-ds.fel.cvut.cz:555/competitions/18/).
For that, following the sumbission instructions, creating a zip file with the docker image and data playback parameters.
- Generate the .tar File from your docker image.
To create the Docker image archive, use the following command:
```
docker save -o my-image.tar my-image-name
```
- (Optional) Create a yaml file with the instructions. 

Example `description.yaml` Content:

(All fields are optional)
```
SENSOR_TRACKS: all                 # Options: all or passive (default: all)
ROSBAG_PLAY_RATE: 5.0             # Default: 5.0 (minimum: 1.0)
```
- Create a `.zip` archive with both `.tar` and `.yaml` file.

**Important**: Make sure both files are located at the same directory level inside the zip.

- The resulting `.zip` file is ready for upload.

###

---

## 🛠 Troubleshooting

If you encounter any issues, refer to the common problems and solutions below.

- **Odometry logger is not writing data**
  - Ensure `/estimated_odom` is correctly published by your SLAM system.
- **Sensors not playing as expected**
  - Check `SENSOR_MODE` in the `.env` file. Ensure it is correctly set.
  - If `passive` is set, LiDAR topics will not be played.
- **RViz is not displaying SLAM output**
  - Run `xhost +` in your terminal before launching the pipeline.

---

## 📜 License

This project is open-source. Feel free to modify and extend it! 🚀

