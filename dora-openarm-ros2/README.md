# dora-openarm-ros2

A [Dora](https://dora-rs.ai/) node that translates OpenArm Dora dataflow messages into [ROS 2](https://www.ros.org/) topics, enabling integration with the ROS 2 ecosystem.

Joint commands from the Dora graph are forwarded to the appropriate ROS 2 controllers, and camera images are published as compressed image topics. If you want to record a dataset, simply run `ros2 bag record` alongside this node to capture all topics.

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
| `camera_wrist_right` | JPEG-encoded image from the right wrist camera as a `uint8[]` array. |
| `camera_wrist_left` | JPEG-encoded image from the left wrist camera as a `uint8[]` array. |
| `camera_head_left` | JPEG-encoded image from the left half of the head stereo camera as a `uint8[]` array. |
| `camera_head_right` | JPEG-encoded image from the right half of the head stereo camera as a `uint8[]` array. |

### ROS 2 Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/left_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Left arm joint trajectory commands. |
| `/right_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Right arm joint trajectory commands. |
| `/left_gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Left gripper trajectory commands. |
| `/right_gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Right gripper trajectory commands. |
| `/camera/wrist_right/image_raw/compressed` | `sensor_msgs/CompressedImage` | Right wrist camera image. |
| `/camera/wrist_left/image_raw/compressed` | `sensor_msgs/CompressedImage` | Left wrist camera image. |
| `/camera/head_left/image_raw/compressed` | `sensor_msgs/CompressedImage` | Left head stereo camera image. |
| `/camera/head_right/image_raw/compressed` | `sensor_msgs/CompressedImage` | Right head stereo camera image. |

## Data Collection

To record all topics for dataset collection, run `ros2 bag record` alongside the dataflow:

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
