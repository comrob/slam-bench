# SLAM Developer Contract (Framework Compatibility)

This document defines the **required interface** for any SLAM container image that should work with this evaluation framework.

If your image satisfies this contract, it is compatible with the orchestration in this repository.

Related documents:
- `README.md` (quick usage and operator workflow)
- `docs/MIGRATION_CONTEXT.md` (architecture and migration details)
- `docs/LLM_PROJECT_RULES.md` (agent maintenance rules)

---

## 1) Purpose

The framework runs three cooperating ROS2 services:

- `run_slam` (your SLAM image)
- `record_odometry` (framework logger)
- `play_bag` (MCAP playback)

Your image is responsible for running SLAM and publishing estimated odometry.

Operational note: playback stage depends on SLAM stage being active (`slam,bag` order).

---

## 2) Hard Requirements

### 2.1 ROS / Runtime

- ROS2 distro: **Jazzy**
- Must run with CycloneDDS environment passed by framework:
  - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - `ROS_DOMAIN_ID` (integer)
  - `ROS_LOCALHOST_ONLY=1`
  - `CYCLONEDDS_URI=file:///config/dds/cyclonedds.xml`
- Must work in `network_mode: host`.

### 2.2 SLAM Process Lifecycle

- The SLAM launch process must stay in the **foreground** and remain alive for the whole bag playback.
- Exiting early (even exit code 0) is treated as failure in practice, because downstream recorder/playback needs SLAM alive.

### 2.3 Config Override Interface (Optional)

- The framework may mount a config file to:
  - `/config/override.yaml`
- Support for this override is **optional**.
- If your SLAM supports runtime config override, it should accept this path (for example `config_override:=/config/override.yaml`).
- If your SLAM does not support it, the system must still start and run with internal defaults.

### 2.4 Required Output Topic

- Must publish:
  - Topic: `/estimated_odom`
  - Type: `nav_msgs/msg/Odometry`

This topic is consumed by the framework logger to produce `estimated_trajectory.txt`.

### 2.5 Output Path Permissions

- The recorder runs as a non-root user (`1000:1000`) and writes trajectory output into the mounted host output directory.
- The host output directory must be writable by that user, otherwise recording fails.

---

## 3) Recommended Launch Contract

Your image should expose one stable launch entry suitable for benchmarking, e.g.:

- `launch/run_lio_sam_bench.py`

Recommended launch behavior:

- Optionally supports argument `config_override` (default `/config/override.yaml`)
- Uses `use_sim_time=true` (because playback publishes `/clock`)
- Starts all required nodes and keeps them alive until SIGINT/SIGTERM

---

## 4) Container Invocation Contract (What the Framework Expects)

In practice, the framework starts your container as `run_slam` from compose and injects:

- Environment variables listed in Section 2.1
- Mounts:
  - `/tmp/.X11-unix:/tmp/.X11-unix` (optional GUI support)
  - `${SLAM_CONFIG_OVERRIDE_FILE}:/config/override.yaml` (optional)
  - `./config/dds/cyclonedds.xml:/config/dds/cyclonedds.xml:ro`

Your image must not assume different mount paths for these interfaces when they are provided.

---

## 5) Image Design Guidelines

### 5.1 Dockerfile

- Base on ROS2 Jazzy image
- Install only runtime/build dependencies needed by your SLAM
- Ensure `ros2 launch` works in container runtime environment
- Prefer deterministic startup (no interactive shell requirements)

### 5.2 Entrypoint / CMD

- For benchmark mode, entrypoint/CMD should run SLAM launch directly, not `sleep infinity`.
- If you support both dev and prod images, make benchmark image default to launching SLAM.

---

## 6) Definition of Done (Compatibility Checklist)

A SLAM image is considered compatible if all items pass:

1. `./orchestrate.sh --stages slam` keeps `run_slam` running (no immediate exit).
2. `./orchestrate.sh --stages slam,bag` runs MCAP playback while SLAM remains active.
3. `ros2 topic echo /estimated_odom` shows valid odometry during playback.
4. Output file `${OUTPUT_PATH_HOST}/estimated_trajectory.txt` is generated and non-empty.
5. `./orchestrate.sh --stages evaluate` completes with trajectory metrics.

---

## 7) Common Integration Failures

- SLAM process starts then exits quickly (container appears to "die").
- Odometry published to a different topic name/type than `/estimated_odom`.
- SLAM requires `/config/override.yaml` to exist and crashes when it is absent.
- Output directory is not writable by UID/GID `1000:1000`, so `record_odometry` cannot create `estimated_trajectory.txt`.
- DDS mismatch (wrong RMW or discovery settings), causing no topic connectivity.

---

## 8) Minimal Validation Commands (Local)

Use these after setting `.env`:

1. Start SLAM only:
  - `./orchestrate.sh --stages slam --follow-slam-logs`
2. In another terminal, verify topic:
   - `docker exec -it run_slam bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list | grep estimated_odom'`
3. Run playback:
   - `./orchestrate.sh --stages bag`
4. Evaluate:
   - `./orchestrate.sh --stages evaluate`

---

## 9) Contract Stability

The following interfaces are considered stable and should be treated as the public compatibility surface:

- `/estimated_odom` (`nav_msgs/msg/Odometry`)
- Optional `/config/override.yaml` override path
- ROS2 Jazzy + CycloneDDS env contract
- Host network ROS2 communication model

If any of these change in the framework, the README and this contract must be updated together.
