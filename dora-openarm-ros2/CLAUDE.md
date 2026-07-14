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

Three dataflow yamls wire together the same VR teleop pipeline (a `udp-receiver` node, pose/trigger/joystick from a Meta Quest headset, package `dora-openarm-vr`; an `ik` node, `dora-openarm-kinematics`, MuJoCo-based IK; and this repo's `dora-to-ros2` node, `build: pip install -e ..`), differing only in the `ik` node's `--mode` and this repo's `--mode` arg:

- `config/dataflow_bridge_ros2_vr.yaml`: **right-arm-only**. `ik` runs `--mode right` (only `--frame-right`/`--frame-type-right`); `dora-to-ros2` currently runs `--mode real`.
- `config/dataflow_bridge_ros2_vr_sim.yaml`: **bimanual**, `dora-to-ros2 --mode sim`.
- `config/dataflow_bridge_ros2_vr_real.yaml`: **bimanual**, `dora-to-ros2 --mode real`.

The two bimanual yamls run `ik --mode bimanual` with both `--frame-right`/`--frame-type-right` and `--frame-left`/`--frame-type-left` set, and wire `target_left`/`trigger_left` (from `udp-receiver`) into `ik`, plus `left_position` (from `ik`) into `dora-to-ros2` — see the bimanual-mode caveat in Architecture point 4 below: with both sides wired, you need to actually use *both* VR controllers, or the solver will stall entirely. A commented-out `mujoco-viewer` node (present in all three) can be re-enabled for visualization.

**`dora-openarm-vr` and `dora-openarm-kinematics` are vendored into `vendor/`** rather than installed via `pip install git+https://...`. Both nodes' `build:` lines point at `pip install -e ../vendor/<name>` instead. This exists because this workspace carries local fixes/patches to those two upstream packages (VR reference-calibration handling, gripper-trigger mapping in `ik.py`, etc.) that are *not* upstreamed — pointing `build:` at the GitHub URL would silently re-fetch pristine upstream on every `dora build`/`uv sync` and wipe them (confirmed happening at least once: `dora-openarm-kinematics` upstream has since renamed its module to `dora_openarm_kinematics` and changed the IK node's pose convention to a bundled float32[8] `[pose, gripper_angle]` instead of separate `target_*`/`trigger_*` inputs — incompatible with this yaml's current wiring, which still expects the older float32[7]-pose-plus-separate-trigger convention). To pull in a genuine upstream update, vendor it deliberately (re-clone into `vendor/<name>`, diff against the previous vendored copy, reapply local patches) rather than switching `build:` back to a git URL.

`config/out/` holds per-run Dora session logs/output (`*.dora-session.yaml`, `log_<node>.txt`) — generated artifacts, not something to hand-edit; only `dataflow_bridge_ros2_vr.dora-session.yaml` is gitignored inside it, so check `git status` before assuming the rest is disposable.

Data collection: run `ros2 bag record` alongside the running dataflow to capture the ROS 2 topics this node publishes (see table in `README.md`).

## Lint / test

```bash
uv run ruff check .
uv run pytest
```

No test files currently exist in this package. `pyproject.toml` enables ruff's `D` (pydocstyle) and `UP` (pyupgrade) rule sets on top of defaults.

## Architecture

`src/dora_ros2_bridge/main.py` is the only wired-up entry point, and is shared by all three dataflow yamls — it adapts to whichever inputs are actually wired rather than hardcoding right-only or bimanual. It supports two mutually exclusive output modes selected via `--mode {sim,real}` (default `sim`; set via the `dora-to-ros2` node's `args:` in the dataflow yaml). On `main()`:

1. Opens a `dora.Ros2Context()` and always creates the `/vr_buttons` (`sensor_msgs/Joy`) publisher, plus mode-specific publisher(s) — see the input/output table in `README.md` for the full mapping.
   - `--mode sim`: single `/joint_command` (`sensor_msgs/JointState`) publisher.
   - `--mode real`: separate `trajectory_msgs/JointTrajectory` publishers per arm, `/left_joint_trajectory_controller/joint_trajectory` and `/right_joint_trajectory_controller/joint_trajectory`.
2. Runs the Dora event loop (`for event in dora.Node()`). `right_position` (`float64[8]`: 7 arm joints + 1 gripper) is cached in module-local `right_cache` and always drives publishing; `left_position`, if wired (the two bimanual yamls), is cached in `left_cache` in the background with no publish of its own — so it's simply whatever was last received by the time the next `right_position` event fires. `left_cache` starts at, and — if `left_position` is never wired (the right-only yaml) — stays at, the constant `LEFT_FIXED` (all joints 0). `button_a`/`button_b`/`button_x`/`button_y` events update `button_cache` (`int32[4]`, order `[a, b, x, y]`) and immediately publish a `Joy` on `/vr_buttons` — messages are built as plain dicts/pyarrow arrays matching ROS 2 message field layouts, not `rclpy` message objects.
   - `sim`: every `right_position` event publishes a merged `JointState` combining `left_cache` and `right_cache` directly (no ramping — fine in simulation).
   - `real`: every `right_position` event instead runs `get_safe_position()` for each arm, which ramps an internal target toward the commanded one by at most `MAX_STEP` (0.02 rad) per call, and resyncs that internal target from real controller feedback first if it's drifted more than `SYNC_THRESHOLD` (0.1 rad) away — real hardware shouldn't be commanded to jump instantly the way simulation can be. Feedback comes from a background `rclpy` thread (`RobotStateSubscriber`, spun via `threading.Thread`) subscribed to both arms' `.../controller_state` topics — this is a second, separate ROS 2 client (`rclpy`) coexisting in-process with the dora Rust-based ROS2 bridge (`dora.Ros2Context`), which is fine since they're independent client library instances.
3. The `sim`-mode `JointState` `name`/`position` order is `openarm_{left,right}_joint1..7` (interleaved) followed by `openarm_left_finger_joint1`, `openarm_right_finger_joint1`. `openarm_left_finger_joint2`, `openarm_left_hand`, `openarm_right_finger_joint2`, `openarm_right_hand`, and both `*_ee_tcp_joint` names (present on `/joint_states`) are intentionally omitted — the Dora inputs only carry one gripper scalar per side, not a value per finger/mimic joint. `real`-mode `JointTrajectory` messages carry only the 7 arm joint names per side (no gripper) — gripper control isn't wired for `real` mode.
4. Bimanual mode's IK solver has an internal `_pending` gate that requires a fresh target from *every* active side before it solves anything at all — this is why the right-only yaml keeps `ik` in `--mode right` (only `right` is in `_pending`) rather than `--mode bimanual`: leaving it bimanual while not using the left controller would stall `position_right` (and therefore all output) forever. The two bimanual yamls intentionally accept this trade-off — both controllers must actually be used.

There is no camera bridging in this node (an earlier revision had `CAMERA_TOPICS`→`sensor_msgs/CompressedImage` publishers; check `git log -p -- src/dora_ros2_bridge/main.py` if you need to reintroduce it). Physical-feedback safety clamping (`get_safe_position`/`RobotStateSubscriber`) *is* present again, but only under `--mode real` — it was ported back from that same git history rather than re-derived from scratch.

**`src/dora_ros2_bridge/openarm_vr_quest_receiver.py` is dead code in this package**: it implements a full Meta Quest UDP pose receiver (LH→RH coordinate transform, `OneEuroPoseSmoother`, reference-pose rectification) but imports `.smoothing` and `.udp_receiver`, neither of which exists in this repo. The real, working version of this node lives in the separate `dora-openarm-vr` package referenced by `config/dataflow_bridge_ros2_vr.yaml`'s `udp-receiver` node, vendored at `vendor/dora-openarm-vr/`. Don't assume this file is reachable or wired into `pyproject.toml` — it has no console-script entry — treat it as reference/leftover, not an active code path, unless asked to revive it.

## Release process

`dev/release.sh <version>` (e.g. `dev/release.sh 1.0.0`) bumps `pyproject.toml`'s version, commits, and tags — it hard-checks that `git remote get-url origin` is `git@github.com:enactic/dora-openarm-ros2.git` before doing anything (this workspace's fork/clone is at a different origin, so the script will refuse to run here without overriding `RELEASE_CHECK_ORIGIN=no`).
