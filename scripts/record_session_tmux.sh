#!/usr/bin/env bash
# Opens the 6-terminal OpenArm Revo2 real-hardware recording session as one tmux window with
# 6 tiled panes, instead of manually opening/cd'ing into 6 separate terminals every time:
#
#   pane 0 (~/pnk/ws)                   -> ros2 launch openarm_bringup ... (bimanual bringup, typed, NOT auto-run)
#   pane 1 (~/pnk/ws/dora-openarm-ros2) -> uv run dora run ... (VR bridge, typed, NOT auto-run)
#   pane 2 (~/pnk/ws)                   -> can_configure (auto) + ros2 launch realsense2_camera ... cam_head (auto-start)
#   pane 3 (~/pnk/ws)                   -> ros2 launch realsense2_camera ... cam_left (auto-start)
#   pane 4 (~/pnk/ws)                   -> ros2 launch realsense2_camera ... cam_right (auto-start)
#   pane 5 (~/pnk/ws)                   -> ros2 bag record ... (typed, NOT auto-run)
#
# Pane 2 auto-runs `can_configure` for both CAN interfaces, then auto-launches cam_head once both
# succeed (chained on one line so bash itself waits out each sudo prompt). Pane 0's bimanual
# bringup command is typed but NOT submitted from the start — press Enter there yourself once
# pane 2 shows both CAN interfaces configured.
#
# Pane 1 (VR bridge) and pane 5 (rosbag record) are also typed but NOT submitted — press Enter in
# each only after confirming (from panes 0/2/3/4's logs) that the CAN arms are up and all 3
# cameras are publishing. This ordering matters: starting the VR bridge or the bag recording too
# early can silently miss real feedback.
#
# Requires tmux (not installed by default on this machine — `sudo apt-get install -y tmux` first).
set -euo pipefail

SESSION="openarm_record"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists — attaching instead of restarting."
  echo "(Run 'tmux kill-session -t $SESSION' first if you want a clean restart.)"
  tmux attach -t "$SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n record -c "$HOME/pnk/ws"
tmux split-window -h -t "$SESSION:0" -c "$HOME/pnk/ws/dora-openarm-ros2"
tmux split-window -v -t "$SESSION:0.0" -c "$HOME/pnk/ws"
tmux split-window -v -t "$SESSION:0.1" -c "$HOME/pnk/ws"
tmux split-window -v -t "$SESSION:0.2" -c "$HOME/pnk/ws"
tmux split-window -v -t "$SESSION:0.4" -c "$HOME/pnk/ws"
tmux select-layout -t "$SESSION:0" tiled

# Pane 0: leave the real-hardware bimanual bringup typed but NOT submitted (wait for pane 2's
# CAN configure to finish first).
tmux send-keys -t "$SESSION:0.0" \
  'ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1 use_fake_hand:=true'

# Pane 1: Dora ROS2<->VR bridge — typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.1" \
  'uv run dora run config/dataflow_bridge_ros2_vr_real.yaml --uv'

# Pane 2: configure both CAN interfaces, then cam_head (auto-starts) — chained on one line so bash
# itself waits out each sudo prompt before launching the camera, instead of racing a second
# send-keys call against an in-progress password prompt.
tmux send-keys -t "$SESSION:0.2" \
  'sudo openarm-can-cli -i can0 can_configure && sudo openarm-can-cli -i can1 can_configure && ros2 launch realsense2_camera rs_launch.py camera_name:=cam_head camera_namespace:=cam_head serial_no:=_243222075840 enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x30 initial_reset:=true' C-m

# Pane 3: cam_left (auto-starts).
tmux send-keys -t "$SESSION:0.3" \
  'ros2 launch realsense2_camera rs_launch.py camera_name:=cam_left camera_namespace:=cam_left serial_no:=_260322277300 enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false depth_module.color_profile:=640x480x30' C-m

# Pane 4: cam_right (auto-starts).
tmux send-keys -t "$SESSION:0.4" \
  'ros2 launch realsense2_camera rs_launch.py camera_name:=cam_right camera_namespace:=cam_right serial_no:=_260322270361 enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false depth_module.color_profile:=640x480x30' C-m

# Pane 5: ros2 bag record of the take-box topic set (real hardware) — typed but deliberately NOT
# submitted.
tmux send-keys -t "$SESSION:0.5" \
  'mkdir -p bags && ros2 bag record -o bags/take_box_$(date +%Y%m%d_%H%M%S) /cam_head/cam_head/color/image_raw /cam_left/cam_left/color/image_raw /cam_right/cam_right/color/image_raw /left_joint_trajectory_controller/controller_state /left_joint_trajectory_controller/joint_trajectory /left_revo2_hand_controller/controller_state /left_revo2_hand_controller/joint_trajectory /right_joint_trajectory_controller/controller_state /right_joint_trajectory_controller/joint_trajectory /right_revo2_hand_controller/controller_state /right_revo2_hand_controller/joint_trajectory /tf /tf_static /vr_buttons /left_ee_pose /right_ee_pose'

tmux attach -t "$SESSION"
