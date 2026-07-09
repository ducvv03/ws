# dora-openarm-ros2

A [Dora](https://dora-rs.ai/) node that translates OpenArm Dora dataflow messages into [ROS 2](https://www.ros.org/) topics, enabling integration with the ROS 2 ecosystem.

Joint commands from the Dora graph are merged into a single `sensor_msgs/JointState` and published on `/joint_command`. If you want to record a dataset, simply run `ros2 bag record` alongside this node to capture all topics.

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
| `left_position` | Left arm joint positions and gripper as a `float64[8]` array (7 joints + 1 gripper). |
| `right_position` | Right arm joint positions and gripper as a `float64[8]` array (7 joints + 1 gripper). |
| `button_a` / `button_b` / `button_x` / `button_y` | VR controller button state (`bool`). |

### ROS 2 Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/joint_command` | `sensor_msgs/JointState` | Merged joint command: `openarm_left_joint1..7`, `openarm_right_joint1..7` (interleaved left/right), then `openarm_left_finger_joint1`, `openarm_right_finger_joint1`. Published whenever either side updates, once both sides have reported at least once. |
| `/vr_buttons` | `sensor_msgs/Joy` | VR controller buttons as `buttons: int32[4]` = `[a, b, x, y]` (0/1), `axes` unused. Published whenever any button input event arrives, holding the latest known state of the other three. |

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
