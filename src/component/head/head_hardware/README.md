# head_hardware

A ros2_control hardware interface (`head_hardware/HeadHw`) for the OpenArm head —
2 joints on 2 separate CAN buses (vertical ↔ left arm bus, horizontal ↔ right arm
bus), Damiao motors via the `openarm_can` library.

## Build

```bash
cd ~/work/ws
source /opt/ros/jazzy/setup.bash

# build only the head plugin
colcon build --packages-select head_hardware

# build together with the description/bringup when you edit the xacro
colcon build --packages-select head_hardware openarm_description pnk_bringup

source install/setup.bash
```

## Launch (bimanual)

The head is **disabled by default**. Enable it with `use_head_vertical` /
`use_head_horizontal`:

```bash
# enable both head joints, real hardware
ros2 launch pnk_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  use_head_vertical:=true \
  use_head_horizontal:=true

# override CAN bus / CAN id at launch time
ros2 launch pnk_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  use_head_vertical:=true  head_vertical_can_interface:=can5  head_vertical_can_id:=0x22 \
  use_head_horizontal:=true head_horizontal_can_interface:=can0 head_horizontal_can_id:=0x23
```

### Head launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_head_vertical` | `false` | enable the vertical head joint |
| `use_head_horizontal` | `false` | enable the horizontal head joint |
| `head_vertical_can_interface` | `can1` | CAN bus for the vertical joint (defaults to the left arm bus) |
| `head_horizontal_can_interface` | `can0` | CAN bus for the horizontal joint (defaults to the right arm bus) |
| `head_vertical_can_id` | `0x22` | CAN id of the vertical motor (decimal or 0x-hex) |
| `head_horizontal_can_id` | `0x23` | CAN id of the horizontal motor |

## Verify (without running the robot)

```bash
# 1) Render the URDF and inspect the head blocks (needs ros2_control:=true)
TOP=src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro
xacro "$TOP" bimanual:=true ros2_control:=true use_fake_hardware:=false \
      use_head_vertical:=true use_head_horizontal:=true \
  | grep -iE "HeadHw|head_joint_|can_interface|can_id|direction"

# 2) Confirm the plugin is registered with pluginlib
cat install/head_hardware/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/head_hardware
#   -> share/head_hardware/head_hardware.xml

# 3) With the robot running: list the hardware components
ros2 control list_hardware_components
```

## Layout / related files

| File | Role |
|---|---|
| `src/head_hardware.cpp`, `include/head_hardware/head_hardware.h` | the `HeadHw : SystemInterface` class |
| `head_hardware.xml` | plugin description (pluginlib) |
| `CMakeLists.txt` | builds the SHARED lib, links `OpenArmCAN::openarm_can`, exports the plugin |
| `.../ros2_control/openarm.bimanual.ros2_control.xacro` | the two head `<ros2_control>` blocks |
| `.../urdf/openarm_v10.urdf.xacro`, `.../robot/openarm_robot.xacro` | thread the head args through the xacro layers |
| `pnk_bringup/launch/openarm.bimanual.launch.py` | declares and passes the head args into xacro |

## CAN bus (reference)

```bash
# bring a CAN interface up (CAN-FD, e.g. can0)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
candump can0          # watch traffic
```
