# dora-openarm-ros2

A [Dora](https://dora-rs.ai/) node that translates OpenArm Dora dataflow messages into [ROS 2](https://www.ros.org/) topics, enabling integration with the ROS 2 ecosystem.

Joint commands from the Dora graph are forwarded to the robot in one of two modes, selected via `--mode {sim,real}` (default `sim`): `sim` merges them into a single `sensor_msgs/JointState` published on `/joint_command`; `real` publishes separate `trajectory_msgs/JointTrajectory` per arm, ramped toward the target and resynced from real controller feedback for hardware safety. If you want to record a dataset, simply run `ros2 bag record` alongside this node to capture all topics.

## Usage

Use this node from a dora-rs dataflow configuration. For a full configuration
example, see
[enactic/dora-openarm-data-collection](https://github.com/enactic/dora-openarm-data-collection).

```yaml
nodes:
  # ...
  - id: dora-to-ros2
    build: pip install dora-openarm-ros2
    path: openarm-dora-ros2
    inputs:
      left_position:      ik/position_left
      right_position:     ik/position_right
      camera_wrist_right: camera-wrist-right/image
      camera_wrist_left:  camera-wrist-left/image
      camera_head_left:   camera-head-stereo-splitter/image_0
      camera_head_right:  camera-head-stereo-splitter/image_1
  # ...
```

### Inputs

| Input | Description |
| --- | --- |
| `left_position` | Left arm joint positions and gripper as a `float64[8]` array (7 joints + 1 gripper; the gripper element is unused, see outputs below). |
| `right_position` | Right arm joint positions and gripper as a `float64[8]` array (7 joints + 1 gripper; the gripper element is unused, see outputs below). |
| `button_a` / `button_b` / `button_x` / `button_y` | VR controller button state (`bool`). |
| `trigger_left` / `trigger_right` | VR controller trigger lever (`float`, `[0, 1]`), drives that hand's 4 non-thumb Revo2 fingers. |
| `joystick_x_left` / `joystick_y_left` / `joystick_x_right` / `joystick_y_right` | VR controller thumbstick axes (`float`, `[-1, 1]`), drives that hand's Revo2 thumb (x → thumb_metacarpal, y → thumb_proximal). |
| `ee_pose_right` / `ee_pose_left` | End-effector pose from an `fk` node — `float32[8]` (or a `{"pose": [...]}` struct) `[px, py, pz, qw, qx, qy, qz, gripper]`. Optional; only used if wired. |

### ROS 2 Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/joint_command` (`--mode sim`, default) | `sensor_msgs/JointState` | Merged joint command: `openarm_left_joint1..7`, `openarm_right_joint1..7` (interleaved left/right), then the left hand's 6 driven Revo2 joints + 5 mirrored `*_distal_joint`s, then the same for the right hand. Published on every `right_position` event. |
| `/left_joint_trajectory_controller/joint_trajectory`, `/right_joint_trajectory_controller/joint_trajectory` (`--mode real`) | `trajectory_msgs/JointTrajectory` | Per-arm arm-only (no gripper) trajectory, one point, `time_from_start` zero (execute immediately). Target is ramped toward at 0.02 rad/cycle and resynced from that controller's `.../controller_state` feedback if it has drifted more than 0.1 rad from what was last commanded. |
| `/left_revo2_hand_controller/joint_trajectory`, `/right_revo2_hand_controller/joint_trajectory` (`--mode real` only) | `trajectory_msgs/JointTrajectory` | 6 Revo2 hand joints per side: `<side>_thumb_metacarpal_joint`, `<side>_thumb_proximal_joint`, `<side>_index_proximal_joint`, `<side>_middle_proximal_joint`, `<side>_ring_proximal_joint`, `<side>_pinky_proximal_joint`. Published on every `trigger_left`/`trigger_right` (fingers) or `joystick_x/y_left`/`joystick_x/y_right` (thumb) event. In `--mode sim` these topics aren't published at all — the same hand state is merged into `/joint_command` instead. |
| `/vr_buttons` | `sensor_msgs/Joy` | VR controller buttons as `buttons: int32[4]` = `[a, b, x, y]` (0/1), `axes` unused. Published whenever any button input event arrives, holding the latest known state of the other three. |
| `/right_ee_pose`, `/left_ee_pose` | `geometry_msgs/PoseStamped` | End-effector translation/orientation, republished from the `ee_pose_right`/`ee_pose_left` inputs whenever they arrive. Not published at all if that input isn't wired in the dataflow yaml. |

## Data Collection

To record all topics for dataset collection, run `ros2 bag record` alongside the dataflow:

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).


python3 -m venv .venv
source .venv/bin/activate
uv run dora build config/dataflow_bridge_ros2_vr.yaml
uv run dora run config/dataflow_bridge_ros2_vr.yaml --uv
