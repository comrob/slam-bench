
# SLAM Pipeline and Evaluation Framework
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository provides a robust and automated framework for running, benchmarking, and evaluating containerized SLAM (Simultaneous Localization and Mapping) systems. It is designed for streamlined use in robotics competitions and research, leveraging Docker and Docker Compose for portability and reproducibility.

### Features

  * **Automated Pipeline:** Execute the entire workflow—running SLAM, playing data, and evaluating results—with a single command.
  * **Containerized & Reproducible:** Runs any Docker-based SLAM system, ensuring consistent environments.
  * **Flexible Configuration:** Easily configure dataset paths, playback parameters, and the target SLAM image via a central `.env` file.
  * **Development Mode:** A dedicated development workflow allows you to build and test local code changes.
  * **Modular Structure:** Cleanly organized source code, entrypoints, and configuration make the framework easy to understand and extend.

### 🏆 Used In

This framework is the official evaluation tool for the following ongoing robotics competitions:
* **SLAM Challenge Competitions:** [https://comrob-ds.fel.cvut.cz:555/?page=1](https://comrob-ds.fel.cvut.cz:555/?page=1)

-----

## 📦 Prerequisites

Ensure you have the following software installed on your system:

  * [Docker Engine](https://docs.docker.com/get-docker/)
  * [Docker Compose](https://docs.docker.com/compose/install/)

-----

## 🚀 Quick Start Guide

1.  **Clone the Repository:**

```bash
git clone [https://github.com/comrob/slam-bench](https://github.com/comrob/slam-bench) -d slam_competition
    cd slam_competition
```

2.  **Configure Your Environment:**

      * Create a new `.env` file (you can copy `.env.example`).
      * Modify `.env` to set the paths to your datasets and specify the Docker image for your SLAM system. See the **Configuration** section below for details.

3.  **Run the Pipeline:**

```bash
./run_pipeline.sh
```

    This script will automatically:

    1.  Start your specified SLAM system and the odometry recorder.
    2.  Play the configured ROS bag file(s).
    3.  Stop all containers after playback.
    4.  Evaluate the final trajectory against the reference.
    5.  Attempt to open the generated PDF report (`trajectory_analysis.pdf`).

-----

## ⚙️ Configuration (`.env` file)

All pipeline parameters are controlled from the `.env` file. Below is a description of the key variables.

| Variable                         | Description                                                                                                                                                             | Example Value                                              |
| -------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| `BAGFILES_PATH_HOST`             | The **absolute path** to the specific dataset directory you want to run.                                                                                                  | `$HOME/bagfiles_competition/shellby-0225-train-lab`        |
| `BAGFILE_NAME`                   | The name of the `.bag` file or a subdirectory within `BAGFILES_PATH_HOST` that contains the `.bag` file(s). Recommended to use `sensors`.                                                        | `sensors`                                                  |
| `SLAM_IMAGE`                     | The name of the Docker image for the SLAM system you want to evaluate. If empty, falls back to `CRL_SLAM_IMAGE`.                                                         | `my-slam-algo:latest`                                      |
| `CRL_SLAM_IMAGE`                 | The default SLAM image to use as a fallback.                                                                                                                              | `ghcr.io/comrob/liorf-crl:latest`                          |
| `REFERENCE_TRAJECTORY_FILE_HOST` | The **absolute path** to the ground truth trajectory file.                                                                                                                | `$BAGFILES_PATH_HOST/reference/reference.txt`              |
| `ROSBAG_PLAY_RATE`               | The playback rate for the `rosbag play` command.                                                                                                                          | `5.0`                                                      |
| `TOPICS_FILE`                    | (Optional) A path relative to `BAGFILES_PATH_HOST` pointing to a file with a newline-separated list of ROS topics to play.                                                 | `tracks/passive.txt`                                       |
| `DEV_DOCKER`                     | Set to `true` to use the locally built `slam-bench:latest` image. **Crucial for the development of *this evaluation repository*, not the SLAM system itself.** | `true`                                                     |
| `SLAM_CONFIG_OVERRIDE_FILE`      | (Optional) A host-side path to a SLAM configuration file that will be mounted into the SLAM container at `/config/override.yaml`.                                        | `./config/slam/override_config.yaml.example`               |

### Expected Dataset Structure for Pipeline

To run the automated `run_pipeline.sh` with a minimal number of variables, the framework expects your dataset to be organized as follows.

````

$BAGFILES\_PATH\_HOST/     \# e.g., /home/user/bagfiles\_competition/shellby-0225-train-lab
├── sensors/
│   └── \*.bag             \# One or more .bag files
├── reference/
│   └── reference.txt     \# The ground truth trajectory
├── calibration/          \# Optional but recommended
│   ├── ...
└── tracks/
   ├── passive.txt       \# Optional topics file
   └── ...

````

**Note:** This structure is recommended for the automated pipeline. When running services manually, you are free to use any layout as long as you provide the correct, full paths in the `.env` file.

-----

## Workflows and Usage

### Development Workflow (for this Repository)

To test changes made to the Python scripts or entrypoints in this evaluation framework:

1.  **Modify the Code** in the `src/` directory.
2.  **Build the local Docker image** using the provided script. This packages your changes into the `slam-bench:latest` image.
```bash
./docker/build.sh
```
3.  **Enable Development Mode** in your `.env` file. This tells Docker Compose to use your locally built image.
```
DEV_DOCKER=true
```
4.  **Run the pipeline** to test your changes in a live environment.
```bash
./run_pipeline.sh
```

### Manual Control

You can run individual components of the pipeline for fine-grained control and debugging. The key is to run the SLAM system and odometry recorder in the background first.

1.  **Start Background Services:** Launch your SLAM system and the odometry recorder. The `-d` flag runs them in detached mode.

```bash
docker compose up -d run_slam record_odometry
```

2.  **Play the Dataset:** Run the `play_bag` service in the foreground. This will stream data to your SLAM system. The command will exit once playback is complete.

```bash
docker compose up play_bag
```

3.  **Evaluate the Trajectory:** **After the bag file has finished playing**, run the evaluation service. This will compare the `estimated_trajectory.txt` generated during the run against the reference.

```bash
docker compose up evaluate_trajectory
```

4.  **Clean Up:** Stop and remove all pipeline containers.

```bash
docker compose down
```

### Visualization with RViz

If your SLAM container publishes visualization markers, you can view them in RViz on your host.

1.  Allow local connections to your X server (run once per session):
    ```bash
    xhost +
    ```
2.  Ensure the `DISPLAY` environment variable is correctly set in your shell.
3.  Run the pipeline. The SLAM container will now connect to your host's display.

-----

## 🏆 Competition Submission

To submit your solution, package your Docker image and an optional description file.

1.  **Save your Docker Image:**
    Archive your final SLAM image into a `.tar` file.

    ```bash
    docker save -o my-slam-image.tar your-slam-image:yourtag
    ```
    Or using the repository script 
    ```
    ./docker/docker2tar.sh your-slam-image:yourtag

2.  **(Optional) Create a `description.yaml`:**
    This file specifies playback parameters. **Check the official competition rules for the required fields and keys.**

    ```yaml
    # Example description.yaml
    ROSBAG_PLAY_RATE: 5.0        # Optional, default: 5.0 - Playback rate for rosbag
    SENSOR_TRACKS: "passive"     # Optional, default: "default" - Sensor track from dataset's tracks/ folder
    # Other parameters as required by the competition...
    ```

    **Available Parameters:**
    - `ROSBAG_PLAY_RATE` (optional, default: 5.0): Controls the playback speed of the ROS bag file
    - `SENSOR_TRACKS` (optional, default: "default"): Specifies which sensor track to use from the dataset's `tracks/` subfolder. Available options include:
      - `"default"` - Uses default set of topics in the bag file
      - Custom sensor tracks defined as `.txt` files in the dataset's `tracks/` directory (e.g., `"passive"` for `tracks/passive.txt`)

3.  **Create the ZIP Archive:**
    Create a `.zip` file containing the `.tar` image archive and the optional `.yaml` file.

    ```bash
    zip submission.zip my-slam-image.tar description.yaml
    ```

    This `submission.zip` file is ready for upload.

-----

## 🛠 Troubleshooting

  * **Evaluation fails or the score is zero:**

      * Check the logs of the `record_odometry` container to see if it's receiving messages on the odometry topic.
      * Verify that your SLAM system is publishing trajectory data correctly.
      * Ensure `$OUTPUT_PATH_HOST/estimated_trajectory.txt` is being created and is not empty.

  * **Local code changes have no effect:**

      * Ensure you have run `./docker/build.sh` after making changes to the code.
      * Verify that `DEV_DOCKER=true` is set in your `.env` file.

  * **Incorrect ROS topics are being played:**

      * Check the `TOPICS_FILE` variable in `.env`. If it's set, ensure the specified file exists at the correct path and contains the desired topic names, one per line.

  * **RViz is not displaying anything:**

      * Make sure you have run `xhost +` on your host machine *before* starting the pipeline.
      * Confirm your `DISPLAY` environment variable is correctly set.

## 📜 License

This project is licensed under the MIT License. See the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.