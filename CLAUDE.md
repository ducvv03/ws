# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository overview

This is a ROS 2 (Jazzy) colcon workspace (`ws`) for a bimanual **OpenArm** robot fitted with **BrainCo Revo2** dexterous hands, plus a VR teleoperation pipeline and a Dora-based data collection bridge. It combines several upstream/vendored ROS packages with project-specific motion-planning and calibration scripts under `src/openarm_motion_planning`.

Layout of `src/`:
- `openarm_ros2/` — upstream OpenArm ROS 2 integration (vendored from enactic/openarm): `openarm_bringup` (launch files, ros2_control controllers, calibration config), `openarm_hardware` (hardware interface, CAN), `openarm_bimanual_moveit_config` (MoveIt config for the dual-arm setup).
- `brainco_hand_ros2/` — upstream BrainCo Revo2 hand packages: `brainco_hardware/brainco_hand_driver` (Modbus/CAN FD/EtherCAT hand driver), `brainco_gazebo`, `brainco_moveit_config`, `revo2_description`, and `revo2_with_rm65_demo` (RM65 arm + Revo2 hand integration demo, unrelated to OpenArm).
- `openarm_description/` — URDF/xacro description for the OpenArm robot (depends on `zed_description` for the head camera mount).
- `openarm_motion_planning/` — **the main project-specific package**. `ament_python` package; the real work lives in `scripts/` (not installed as console entry points — run directly with `python3 src/openarm_motion_planning/scripts/<name>.py`). Scripts talk to MoveIt directly via `MoveGroup` action, `/compute_ik`, and `/apply_planning_scene` — they do not use `openarm_motion_planning/openarm_motion_planning_client.py`, which is dead/stale code left over from a different (non-OpenArm) robot project and imports packages (`vm_per_motion_planning_msgs`, `vm_robot_demo`) that don't exist in this workspace.
- `openarmx_teleop_vr/` — VR teleoperation pipeline: `openarmx_teleop_bridge_vr` (C++, UDP → ROS 2 topics/TF from the VR headset/controllers) and `openarmx_teleop_vr` (Python, subscribes to bridge topics + `/joint_states`, does IK, publishes to `/left_forward_position_controller/commands` and `/right_forward_position_controller/commands`).
- `pnk_perception_msgs/` — small `ament_cmake` interface package defining `GraspTarget.msg` (AprilTag-relative grasp poses: center/pre-left/pre-right/left/right).
- `dora-openarm-ros2/` (top-level, separate from `src/`) — a [Dora](https://dora-rs.ai/) node/Python package (managed with `uv`, not colcon) that bridges Dora dataflow messages to ROS 2 topics for teleop data collection. Build/run with `uv run dora build/run config/dataflow_bridge_ros2_vr.yaml`, not colcon.

## Build

Standard colcon workspace. From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package (faster iteration):

```bash
colcon build --packages-select openarm_motion_planning
colcon build --packages-up-to openarm_bringup
```

`build/`, `install/`, and `log/` are colcon-generated and gitignored — never hand-edit files there.

## Running things

Bring up the real bimanual arms (CAN interfaces must be configured first with `openarm-can-cli -i <can> can_configure`):

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1 use_fake_hand:=true
```

Fake-hardware/simulation variant (no physical robot needed):

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true use_fake_hand:=true
```

MoveIt planning stack:

```bash
ros2 launch openarm_bimanual_moveit_config move_group.launch.py
```

Motion planning / grasping / calibration scripts (run as plain Python scripts against a live ROS 2 graph, not via `ros2 run`):

```bash
python3 src/openarm_motion_planning/scripts/demo4.py
python3 src/openarm_motion_planning/scripts/grasp_generation.py
```

VR teleoperation (start in this order — robot base, then bridge, then teleop node, to avoid failures from topics not yet existing):

```bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

RealSense camera (used for AprilTag-based grasp target detection):

```bash
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true pointcloud.enable:=true align_depth.enable:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30
```

## Tests / lint

`openarm_motion_planning` and `openarm_description` use standard ament test hooks (`ament_flake8`, `ament_pep257`, `ament_copyright`, `ament_lint_auto`/`ament_lint_common`):

```bash
colcon test --packages-select openarm_motion_planning
colcon test-result --verbose
```

## Conventions specific to this repo

- Per-object grasp/manipulation parameters (tag size, offset quaternions, grasp waypoints, obstacle geometry, hand close pose) live in `src/openarm_motion_planning/config/config.yaml`, keyed by AprilTag ID (e.g. `id_10` for the rectangle box). New object types should be added there rather than hardcoded in scripts.
- `openarm_motion_planning/scripts/` has many numbered `demoN.py`/`grasp_generationN.py` variants — these are iterative snapshots, not a versioned API. When asked to fix or extend "the demo"/"grasp generation", check `git log` and the current branch to determine which numbered file is the one actively being worked on before assuming it's the highest number.
- Joint-state and trajectory CSVs at the workspace root (`joint_states_*.csv`, `trajectory.csv`, `planned_trajectory.csv`, `wave_right.csv`, `full_flow_test_result.csv`) are recorded/replayed data from `save_jointstates.py`/`load_jointstates.py`-style scripts, not fixtures to edit by hand.
- Frame/link names follow `openarm_body_link0` (base), `left_gripper_tcp`/`right_gripper_tcp` (Revo2 hand tool frames), and controllers are named `<left|right>_joint_trajectory_controller`, `<left|right>_forward_position_controller`, `<left|right>_revo2_hand_controller`.
