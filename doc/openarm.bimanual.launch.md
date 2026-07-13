# `openarm.bimanual.launch.py` — Launch Arguments

Variables that can be set at launch:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py <arg>:=<value> ...
```

| Argument | Default | Choices | Description |
|---|---|---|---|
| `description_package` | `openarm_description` | — | Package with the robot URDF/xacro files. |
| `description_file` | `v10.urdf.xacro` | — | URDF/xacro description file. |
| `arm_type` | `openarm_v1.0` | `v1.0`, `v10`, `openarm_v1.0`, `v2.0`, `v20`, `openarm_v2.0` | Arm type. |
| `use_fake_hardware` | `true` | `true` / `false` | Use fake hardware instead of real CAN. Set `false` for real motors. |
| `use_fake_hand` | `true` | `true` / `false` | Use fake hand instead of the real hand. |
| `use_teleop` | `false` | `true` / `false` | Use the teleop-to-sim hardware interface (`OpenArmHWTeleOp`) for the arms. |
| `right_can_interface` | `can0` | — | CAN interface for the right arm. |
| `left_can_interface` | `can1` | — | CAN interface for the left arm. |
| `use_head_vertical` | `false` | `true` / `false` | Enable the vertical head joint (`head_hardware/HeadHw`). |
| `use_head_horizontal` | `false` | `true` / `false` | Enable the horizontal head joint (`head_hardware/HeadHw`). |
| `head_vertical_can_interface` | `can1` | — | CAN interface for the vertical head joint (defaults to the left arm bus). |
| `head_horizontal_can_interface` | `can0` | — | CAN interface for the horizontal head joint (defaults to the right arm bus). |
| `head_vertical_can_id` | `0x22` | — | CAN id of the vertical head motor (decimal or 0x-hex). |
| `head_horizontal_can_id` | `0x23` | — | CAN id of the horizontal head motor (decimal or 0x-hex). |
| `robot_controller` | `joint_trajectory_controller` | `forward_position_controller`, `joint_trajectory_controller` | Arm controller to start. |
| `runtime_config_package` | `openarm_bringup` | — | Package holding the controller config folder. |
| `controllers_file` | `openarm_bimanual_controllers.yaml` | — | Controllers config file (also contains the head controller). |
| `arm_prefix` | `""` (empty) | — | Namespace prefix for topics. Empty = no namespace (`/controller_manager`). |

> **Note:** `head_forward_position_controller` is a combined controller that claims both
> head joints, so `use_head_vertical` and `use_head_horizontal` must both be `true`
> together — enabling only one leaves a missing interface and the controller fails to
> activate.
