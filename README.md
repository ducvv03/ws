sudo openarm-can-cli -i can0 can_configure
sudo openarm-can-cli -i can1 can_configure

cd ~/openarm_ros2_ws
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1

ros2 launch openarm_bimanual_moveit_config move_group.launch.py

cd ~/pnk/pnk_ws
python3 src/openarm_motion_planning/scripts/demo4.py


teleop


python3 -m venv .venv
source .venv/bin/activate
uv run dora build config/dataflow_bridge_ros2_vr.yaml
uv run dora run config/dataflow_bridge_ros2_vr.yaml --uv


[//]: # (ros2 launch openarm_bringup openarm_vr.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1)

[//]: # ()
[//]: # (cd ~/pnk/pnk_ws)

[//]: # (ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node)

[//]: # ()
[//]: # (cd ~/pnk/pnk_ws)

[//]: # (ros2 launch openarmx_teleop_vr teleop_vr.launch.py)

[//]: # ()
[//]: # (cd ~/pnk/pnk_ws)

[//]: # (python3 src/openarm_motion_planning/scripts/pub.py)