# ROS1 Bagfile Player with Odometry Logger

This project provides a **Dockerized** solution for playing **ROS1 bagfiles**, publishing relevant topics to the host system, and recording odometry data from the `/estimated_odom` topic. The logged trajectory is stored in a CSV file for subsequent analysis. 

Additionally, the project includes a trajectory evaluation service that computes RPE (Relative Pose Error) and ATE (Absolute Trajectory Error) using ground truth data, facilitating performance assessment of odometry estimations.

---

## 📁 Project Structure
```
├── docker-compose.yaml
├── Dockerfile
├── entrypoint_evaluate.sh
├── entrypoint.sh
├── README.md
└── scripts
    ├── odometry_logger.py
    ├── trajectory_analysis_host.ipynb
    └── trajectory_analysis.py
```

---

## 🚀 Quick Start

### Specify the Bagfiles

In the `.env` file, specify:
- `BAGFILES_PATH_HOST`: The path to the directory containing the bagfiles dataset on the host machine.
- `BAGFILE_NAME`: The name of the bagfile to play relative to the `BAGFILES_PATH_HOST`.
- `GT_FILE`: The path to the ground truth file relative to the `BAGFILES_PATH_HOST`.

Example:
```bash
BAGFILES_PATH_HOST=/mnt/storage/bags/shellbe_2024-02-21/final/shellbeLabCalibration
BAGFILE_NAME=sensors_shellbeLabCalibration_2025-02-25-16-53-00.bag
GT_FILE=reference/totalStation_shellbeLabCalibration_2025-02-25-16-52-55_reference.csv
```

### 1️⃣ **Build the Docker Image**
Run the following command to build the Docker container:

```bash
docker compose build
```

---

### 2️⃣ **Run the Dataset Evaluation**
First, run the SLAM on your machine. Ensure that the estimated odometry is published to the `/estimated_odom` topic.

Then, execute the following command to process the dataset and log the estimated odometry:

```bash
docker compose up run_bag
```

✔ This will:
- Start `roscore`
- Run the **odometry logger** before playing the bagfile
- Play the bagfile while logging `/estimated_odom` to `$BAGFILES_PATH_HOST/evaluation_output/estimated_trajectory.csv`

---

### 3️⃣ **Verify Logged Data**
After stopping the container, check the logged trajectory on your **host machine**:

```bash
cat $BAGFILES_PATH_HOST/evaluation_output/estimated_trajectory.csv
```

📌 **Expected CSV format** (TUM format):

```
timestamp x y z qw qx qy qz (no header)
```

---

## 🛠 **Troubleshooting**
  
- **Odometry logger not writing data**  
  - Check that `/estimated_odom` is published by your SLAM system.

### 4️⃣ **Evaluate the Trajectory**

- Specify the ground truth file in the `.env` file.

It should be relative to the `BAGFILES_PATH_HOST`.
Example:
```bash
GT_FILE=reference/totalStation_shellbeLabCalibration_2025-02-25-16-52-55_reference.csv
```

For datasets under 50 meters, use:
```bash
TEST_MODE=1
```
Otherwise, use:
```bash
TEST_MODE=0
```
This will switch between delta sets for RPE calculation.

Run the following command to evaluate the trajectory:

```bash
docker compose up evaluate_trajectory
```

The output will be saved to the `evaluation_output` directory in the `BAGFILES_PATH_HOST`.
It will contain the following files:
- `estimated_trajectory.csv`: The estimated trajectory from the odometry logger.
- `trajectory_analysis.pdf`: A PDF file containing trajectory plots and evaluation metrics.
- `trajectory_analysis.yaml`: A YAML file containing trajectory evaluation metrics.

## 📜 **License**
This project is open-source. Feel free to modify and extend it for your use! 🚀

