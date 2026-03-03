# LLM Project Rules and Current State

This file is the authoritative guide for any LLM/automation agent editing this repository.

Related documentation:

- `README.md`
- `docs/SLAM_DEVELOPER_CONTRACT.md`
- `docs/MIGRATION_CONTEXT.md`
- `docs/changelog/AGENT_CHANGELOG.md`
- `docs/changelog/WIP_CHANGE_CONTEXT.md`

---

## 1) Current Project State (as of 2026-03-03)

- ROS stack migrated to **ROS2 Jazzy**.
- DDS strategy is fixed to **CycloneDDS loopback-only** (localhost RAM path), configured via `CYCLONEDDS_URI` and mounted XML.
- Dataset playback is **MCAP-only** (`ros2 bag play`), with support for:
  - collection directories containing `metadata.yaml`, and
  - direct single `.mcap` file mode for fast validation.
- Orchestration is modular:
  - library: `lib/orchestration.sh`
  - CLI: `orchestrate.sh`
  - compatibility wrapper: `run_pipeline.sh`
- `slam` stage starts `run_slam` + `record_odometry`.
- `bag` stage requires SLAM already running.
- `--follow-slam-logs` streams SLAM/recorder logs.
- Recorder writes trajectory as non-root user (UID/GID `1000:1000`); host output directory must be writable.

---

## 2) Public Compatibility Contracts

When editing behavior, preserve these interfaces unless explicitly requested otherwise:

1. Topic contract:
   - `/estimated_odom` with type `nav_msgs/msg/Odometry`
2. Optional config override mount path:
   - `/config/override.yaml`
3. ROS runtime contract:
   - Jazzy + `rmw_cyclonedds_cpp` + host networking + localhost-only policy
4. Orchestration CLI contract:
   - `orchestrate.sh --stages ...`

If any of these must change, update all documentation and call out breaking changes.

---

## 3) Mandatory Editing Rules

On **every code/config change**, the agent must:

1. Update documentation to reflect actual behavior:
   - `README.md`
   - `docs/SLAM_DEVELOPER_CONTRACT.md`
   - `docs/MIGRATION_CONTEXT.md`
2. Update this file (`docs/LLM_PROJECT_RULES.md`) when project state, workflow, or rules change.
3. Keep docs synchronized with implementation (no stale flags, commands, or stage behavior).
4. Avoid introducing hidden behavior changes without documentation.
5. Preserve minimal configurability principle unless user asks for more options.
6. Maintain `docs/changelog/WIP_CHANGE_CONTEXT.md` during implementation with current rationale, progress, and approach details.
7. Append a concise entry to `docs/changelog/AGENT_CHANGELOG.md` **only after**:
   - changes are tested/validated, and
   - user approves the changes.
8. Do **not** append final changelog entries during the implementation stage.

If a change does not affect behavior, documentation updates can be noted as "no behavioral doc changes required".

---

## 4) Orchestration-Specific Rules

- Stage order matters; do not assume commutativity:
  - valid flow: `slam,bag[,evaluate]`
- If adding blocking log streaming, ensure multi-stage execution still proceeds.
- Fail fast with actionable errors for:
  - missing running SLAM before `bag`
  - non-writable output path for recorder
- Keep cleanup behavior explicit and predictable.

---

## 5) Validation Rules for Agents

After modifying scripts or compose behavior, run relevant checks when possible:

1. Shell syntax:
   - `bash -n orchestrate.sh lib/orchestration.sh`
2. Compose rendering:
   - `docker compose -f docker-compose.yaml -f docker-compose-dev.yaml config`
3. Stage smoke tests:
   - `./orchestrate.sh --stages slam`
   - `./orchestrate.sh --stages bag`

If execution cannot be completed (e.g., image unavailable), document what was validated and what remains.

---

## 6) Change Log Discipline

For each approved and tested agent-driven edit set, include in user summary:

- what changed,
- why it changed,
- which files were updated,
- what validation was performed,
- any remaining known limitations.

Implementation-phase notes belong in `docs/changelog/WIP_CHANGE_CONTEXT.md` until approval.
