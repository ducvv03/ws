# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository overview

This is a ROS 2 (Jazzy) colcon workspace (`ws`) for a bimanual **OpenArm** robot fitted with **BrainCo Revo2** dexterous hands and a 2-DOF head, plus a VR teleoperation pipeline and a Dora-based data collection bridge. The robot as a whole is called **PNK**.

`src/` is organised in four layers. **Each layer may only depend on the layer below it** — never sideways, never upward. Descriptions (URDF/xacro/meshes) all live under `system/pnk_description/`; `component/` holds hardware interfaces and drivers only.

- `lib/` — plain libraries, no ROS node or plugin.
  - `openarm_can` — enactic SocketCAN library, **vendored in full** into this repo (no submodule, no `.gitignore` exclusion) so a fresh clone builds. It carries local changes to the Damiao motor code on top of upstream `a303646` (v1.3.4); pulling from `github.com/enactic/openarm_can` now means a manual merge.
  - `brainco_stark_sdk_vendor` — downloads the prebuilt BrainCo Stark SDK zip at build time and installs it into `install/`. Exports the imported target `brainco_stark_sdk::brainco_stark_sdk`. Version via `-DBRAINCO_STARK_SDK_VERSION=` (default `v1.0.0`).
- `component/` — one hardware subsystem each: hardware interface and driver.
  - `component/openarm/` — `openarm_hardware` (ros2_control interface + CAN; the vendored `openarm_teleop` project is still nested inside it), `openarm_pid_controller`. Hardware only — the arm's URDF lives in `system/pnk_description/`.
  - `component/brainco/` — `brainco_hand_driver` (Modbus/SocketCAN/CAN FD hand driver), `brainco_moveit_config`, `brainco_gazebo`, and `revo2_with_rm65_demo` (RM65 arm demo, unrelated to this robot).
  - `component/head/` — `head_hardware` (2 Damiao motors on separate CAN buses).
- `system/` — wires the components into one robot, and owns every robot description.
  - `pnk_description/` — **a plain folder, not a package**, grouping the two description packages:
    - `openarm_description` — everything about the robot's shape *and* its assembly, kept together the way it always was: `assets/robot/openarm_v1.0/urdf/` holds `arm/`, `body/`, `ee/`, `head/` (the parts) alongside `openarm_v10.urdf.xacro`, `robot/openarm_robot.xacro` and `ros2_control/` (the assembly that welds arms + Revo2 hands + head into one robot and picks the hand ros2_control backend). `openarm_v2.0/` is the v2.0 arm assembly, currently bare, kept so hands and head can be added in place when the hardware is upgraded. Also holds the display launch and the rviz configs.
    - `revo2_description` — Revo2 hand URDF and meshes, vendored in full from upstream BrainCoTech `8ca54d2` (was an orphan gitlink with no `.gitmodules`; de-submoduled so a clone actually gets the meshes).
  - `pnk_bringup` — the launch entry point, `ros2_control` controller config for arms + hands + head, calibration.
  - `pnk_moveit_config` — MoveIt config covering both arms **and** both Revo2 hands.
  - `pnk` — metapackage.
- `app/` — uses the robot to do work.
  - `openarm_motion_planning` — `ament_python`; the real work lives in `scripts/` (not console entry points — run directly with `python3 src/app/openarm_motion_planning/scripts/<name>.py`). Scripts talk to MoveIt via the `MoveGroup` action, `/compute_ik` and `/apply_planning_scene`; they do **not** use `openarm_motion_planning_client.py`, which is dead code from a different robot project.
  - `teleop_vr/` — `openarmx_teleop_bridge_vr` (C++, UDP → ROS 2 topics/TF from the headset) and `openarmx_teleop_vr` (Python, IK, publishes to the forward position controllers).
  - `pnk_perception_msgs` — `GraspTarget.msg` (AprilTag-relative grasp poses).
  - `denso_app` — web console for motor telemetry and arm enable/disable. Only `mock/` is temporary — a Python stand-in for a C++ server to come; `frontend/` (static HTML/CSS/JS, no build step) and `API.md` stay put when it is replaced. **The server interprets nothing** — it forwards ros2_control interface names verbatim with an arrival stamp; thresholds, staleness, severity and layout all live in `frontend/config.js`. One exception, deliberately: the enable interlock (HTTP 409) stays server-side. Layering inside the mock: `http_api/` never imports rclpy, `telemetry/`+`control/` never import fastapi, only `main.py` sees both. Run with `ros2 run denso_app mock_server`; needs `python3-fastapi python3-uvicorn` from apt. Reads `/dynamic_joint_states`, so any state interface the hardware starts exporting appears with no code change.
- `dora-openarm-ros2/` (top-level, outside `src/`) — a [Dora](https://dora-rs.ai/) node/Python package (managed with `uv`, not colcon) bridging Dora dataflow to ROS 2 for teleop data collection. Build/run with `uv run dora build/run config/dataflow_bridge_ros2_vr.yaml`.

Naming rule: **path tells you the layer, prefix tells you the owner.** `pnk_` is code written for this project; upstream names (`openarm_`, `brainco_`, `revo2_`) are kept as-is so those packages can still be merged from upstream. Several packages still carry upstream names pending a rename — see "Pending" below.

EtherCAT support and all test/lint hooks were deliberately removed; this robot runs SocketCAN (or Modbus) only.

## Build

Standard colcon workspace. From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package (faster iteration):

```bash
colcon build --packages-select openarm_motion_planning
colcon build --packages-up-to pnk_bringup
```

`build/`, `install/`, and `log/` are colcon-generated and gitignored — never hand-edit files there.

## Running things

Bring up the real bimanual arms (CAN interfaces must be configured first with `openarm-can-cli -i <can> can_configure`):

```bash
ros2 launch pnk_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1 use_fake_hand:=true
```

Fake-hardware/simulation variant (no physical robot needed):

```bash
ros2 launch pnk_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true use_fake_hand:=true
```

MoveIt planning stack:

```bash
ros2 launch pnk_moveit_config move_group.launch.py
```

Motion planning / grasping / calibration scripts (run as plain Python scripts against a live ROS 2 graph, not via `ros2 run`):

```bash
python3 src/app/openarm_motion_planning/scripts/demo4.py
python3 src/app/openarm_motion_planning/scripts/grasp_generation.py
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

There are none. Every `if(BUILD_TESTING)` block, `<test_depend>` and test directory was removed on purpose — `colcon test` does nothing here. Don't add lint hooks back unless asked.

## Conventions specific to this repo

- Per-object grasp/manipulation parameters (tag size, offset quaternions, grasp waypoints, obstacle geometry, hand close pose) live in `src/app/openarm_motion_planning/config/config.yaml`, keyed by AprilTag ID (e.g. `id_10` for the rectangle box). New object types should be added there rather than hardcoded in scripts.
- `openarm_motion_planning/scripts/` has many numbered `demoN.py`/`grasp_generationN.py` variants — these are iterative snapshots, not a versioned API. When asked to fix or extend "the demo"/"grasp generation", check `git log` and the current branch to determine which numbered file is the one actively being worked on before assuming it's the highest number.
- Joint-state and trajectory CSVs at the workspace root (`joint_states_*.csv`, `trajectory.csv`, `planned_trajectory.csv`, `wave_right.csv`, `full_flow_test_result.csv`) are recorded/replayed data from `save_jointstates.py`/`load_jointstates.py`-style scripts, not fixtures to edit by hand.
- Frame/link names follow `openarm_body_link0` (base), `left_gripper_tcp`/`right_gripper_tcp` (Revo2 hand tool frames), and controllers are named `<left|right>_joint_trajectory_controller`, `<left|right>_forward_position_controller`, `<left|right>_revo2_hand_controller`.
