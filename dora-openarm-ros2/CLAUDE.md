# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`dora-openarm-ros2` is a [Dora](https://dora-rs.ai/) dataflow node (Python package `dora_ros2_bridge`, published as `dora-openarm-ros2`) that bridges a Dora dataflow graph to ROS 2 topics for the bimanual OpenArm robot. It is a standalone `uv`-managed Python project living inside the `ws` colcon workspace (`ws/dora-openarm-ros2/`) — it is **not** a colcon/ament package and is not built with `colcon build`. See `/home/ws/pnk/ws/CLAUDE.md` for how this directory fits into the broader workspace.

The console entry point is `openarm-dora-ros2 = dora_ros2_bridge.main:main` (declared in `pyproject.toml`), run as the `dora-to-ros2` node in a Dora dataflow YAML.

## Environment / dependency management

Uses `uv`, not pip/colcon:

```bash
uv sync                 # install/update the venv from uv.lock
uv run <command>         # run a command inside the project venv
```

Core runtime deps: `dora-rs`, `dora-rs-cli` (both pinned to `0.5.0`), `pyarrow`. `rclpy`, `control_msgs`, `numpy`, and `scipy` are consumed by the node code but are provided by the ROS 2/OS Python environment rather than `pyproject.toml` — this package expects to run where a sourced ROS 2 install is already on `PYTHONPATH`.

Dev deps (`ruff==0.14.11`, `pytest`) are in the `dev` dependency group; ruff version must stay in sync with `.pre-commit-config.yaml` if one is added.

## Running the dataflow

Build and run via `dora`, from this directory, not colcon:

```bash
uv run dora build config/dataflow_bridge_ros2_vr.yaml
uv run dora run config/dataflow_bridge_ros2_vr.yaml
```

`config/dataflow_bridge_ros2_vr.yaml` wires together the VR teleop pipeline: a `udp-receiver` node (pose/trigger/joystick from a Meta Quest headset, package `dora-openarm-vr`, installed separately), an `ik` node (`dora-openarm-kinematics`, bimanual MuJoCo-based IK), and this repo's `dora-to-ros2` node (`build: pip install -e ..`, i.e. this package installed editable). A commented-out `mujoco-viewer` node can be re-enabled for visualization.

`config/out/` holds per-run Dora session logs/output (`*.dora-session.yaml`, `log_<node>.txt`) — generated artifacts, not something to hand-edit; only `dataflow_bridge_ros2_vr.dora-session.yaml` is gitignored inside it, so check `git status` before assuming the rest is disposable.

Data collection: run `ros2 bag record` alongside the running dataflow to capture the ROS 2 topics this node publishes (see table in `README.md`).

## Lint / test

```bash
uv run ruff check .
uv run pytest
```

No test files currently exist in this package. `pyproject.toml` enables ruff's `D` (pydocstyle) and `UP` (pyupgrade) rule sets on top of defaults.

## Architecture

`src/dora_ros2_bridge/main.py` is the only wired-up entry point. Current configuration is **right-arm-only teleop** — the left arm/gripper are not driven by the Dora graph. On `main()`:

1. Opens a `dora.Ros2Context()` and creates two ROS 2 publishers: `/joint_command` (`sensor_msgs/JointState`) and `/vr_buttons` (`sensor_msgs/Joy`) — see the input/output table in `README.md` for the full mapping.
2. Runs the Dora event loop (`for event in dora.Node()`). Only `right_position` (`float64[8]`: 7 arm joints + 1 gripper) is consumed and cached in module-local `right_cache`; the left side is always published as the constant `LEFT_FIXED` (all joints 0). Every `right_position` event publishes a merged `JointState` combining `LEFT_FIXED` and `right_cache`. `button_a`/`button_b`/`button_x`/`button_y` events update `button_cache` (`int32[4]`, order `[a, b, x, y]`) and immediately publish a `Joy` on `/vr_buttons` — messages are built as plain dicts/pyarrow arrays matching ROS 2 message field layouts, not `rclpy` message objects.
3. The published `JointState` `name`/`position` order is `openarm_{left,right}_joint1..7` (interleaved) followed by `openarm_left_finger_joint1`, `openarm_right_finger_joint1`. `openarm_left_finger_joint2`, `openarm_left_hand`, `openarm_right_finger_joint2`, `openarm_right_hand`, and both `*_ee_tcp_joint` names (present on `/joint_states`) are intentionally omitted — the Dora inputs only carry one gripper scalar per side, not a value per finger/mimic joint.
4. `config/dataflow_bridge_ros2_vr.yaml`'s `ik` node runs in `--mode right` (not `bimanual`) to match: in bimanual mode the IK solver's internal `_pending` gate requires a fresh left target before it solves *anything*, so leaving it in bimanual mode while not using the left controller would stall `position_right` forever.

There is no camera bridging and no physical-feedback safety clamping in this node (an earlier revision had both: `CAMERA_TOPICS`→`sensor_msgs/CompressedImage` publishers, and a `get_safe_position` step-limiter driven by an `rclpy` `RobotStateSubscriber`). If you need to reintroduce either, check `git log -p -- src/dora_ros2_bridge/main.py` for that prior implementation rather than re-deriving it from scratch.

**`src/dora_ros2_bridge/openarm_vr_quest_receiver.py` is dead code in this package**: it implements a full Meta Quest UDP pose receiver (LH→RH coordinate transform, `OneEuroPoseSmoother`, reference-pose rectification) but imports `.smoothing` and `.udp_receiver`, neither of which exists in this repo. The real, working version of this node lives in the separate `dora-openarm-vr` package referenced by `config/dataflow_bridge_ros2_vr.yaml`'s `udp-receiver` node (`pip install git+https://github.com/enactic/dora-openarm-vr`). Don't assume this file is reachable or wired into `pyproject.toml` — it has no console-script entry — treat it as reference/leftover, not an active code path, unless asked to revive it.

## Release process

`dev/release.sh <version>` (e.g. `dev/release.sh 1.0.0`) bumps `pyproject.toml`'s version, commits, and tags — it hard-checks that `git remote get-url origin` is `git@github.com:enactic/dora-openarm-ros2.git` before doing anything (this workspace's fork/clone is at a different origin, so the script will refuse to run here without overriding `RELEASE_CHECK_ORIGIN=no`).
