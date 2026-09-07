#!/usr/bin/env bash
# Companion to record_session_tmux_auto.sh — splitting what used to be one 9-pane tmux window
# (which hit tmux's "no space for new pane" error on smaller terminals) into two separate tmux
# sessions you run side by side: this one holds every pane whose main command is only typed in,
# waiting for you to press Enter yourself, in its own 4-pane window:
#
#   pane 0 (ssh JetsonAGX@10.87.25.239, ~/pnk/ws) -> ros2 launch pnk_bringup ... (bimanual bringup, typed, NOT auto-run)
#   pane 1 (~/pnk/ws/dora-openarm-ros2, local)    -> venv setup (auto) + uv run dora run ... (VR bridge, typed, NOT auto-run)
#   pane 2 (ssh JetsonAGX@10.87.25.239, ~/data)   -> ros2 bag record ... (typed, NOT auto-run)
#   pane 3 (local, transrecv_udp)                 -> VR connect client (typed, NOT auto-run)
#
# See record_session_tmux_auto.sh for the other 5 panes (CAN configure + cameras, NAS mount,
# transrecv_udp server) — start that script FIRST and let it run for a bit before pressing Enter
# in any of this session's panes:
#   - Pane 0 here needs record_session_tmux_auto.sh's pane 0 (`can_configure`) to finish first.
#   - Pane 2 here (bag record, `cd ~/data`) needs record_session_tmux_auto.sh's pane 3 (NAS sshfs
#     mount) to finish first, or recordings land on local disk instead of the NAS.
#
# SSH login is automated via `sshpass` (requires `sudo apt-get install -y sshpass`) using the
# Jetson's known password, so no manual password entry is needed for the SSH hop itself. There can
# still be a brief connection-establishment delay before the remote shell is ready, though — if a
# pane doesn't show its next command ready right after connecting, just retype it.
#
# SECURITY NOTE: the Jetson's SSH password is embedded in this script (JETSON_PASS below) for
# automation — this repo is pushed to a GitHub remote, so that password is effectively public in
# git history from this commit onward. Prefer switching to SSH key-based auth for the Jetson and
# dropping sshpass/JETSON_PASS entirely if this matters.
#
# Press Enter in each pane only after confirming (from record_session_tmux_auto.sh's panes) that
# the CAN arms are up, all 3 cameras are publishing, and (for bag recording) the NAS is mounted.
# This ordering matters: starting the VR bridge, the connect client, or the bag recording too
# early can silently miss real feedback.
#
# Requires tmux (not installed by default on this machine — `sudo apt-get install -y tmux` first).
set -euo pipefail

SESSION="openarm_record_manual"
JETSON="JetsonAGX@10.87.25.239"
JETSON_PASS="1"
SSH_JETSON="sshpass -p '$JETSON_PASS' ssh $JETSON"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists — attaching instead of restarting."
  echo "(Run 'tmux kill-session -t $SESSION' first if you want a clean restart.)"
  tmux attach -t "$SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n manual -c "$HOME/pnk/ws"
tmux split-window -h -t "$SESSION:0" -c "$HOME/pnk/ws/dora-openarm-ros2"
tmux split-window -v -t "$SESSION:0.0" -c "$HOME/pnk/ws"
tmux split-window -v -t "$SESSION:0.1" -c "/home/ws/pnk/data_collection/simple_trans_receive/transrecv_udp"
tmux select-layout -t "$SESSION:0" tiled

# Pane 0: SSH into the Jetson, cd to ~/pnk/ws (auto-starts), then leave the real-hardware bimanual
# bringup typed but NOT submitted (wait for record_session_tmux_auto.sh's CAN configure to finish
# first).
tmux send-keys -t "$SESSION:0.0" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.0" \
  'cd ~/pnk/ws' C-m
tmux send-keys -t "$SESSION:0.0" \
  'ros2 launch pnk_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1 use_fake_hand:=true'

# Pane 1 (local, unchanged): set up the venv (auto-starts), then leave the Dora ROS2<->VR bridge
# command typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.1" \
  'cd ~/pnk/ws/dora-openarm-ros2/ && python3 -m venv .venv && source .venv/bin/activate' C-m
tmux send-keys -t "$SESSION:0.1" \
  'uv run dora run config/dataflow_bridge_ros2_vr_real.yaml --uv'

# Pane 2: SSH into the Jetson, cd to ~/data (auto-starts — should already be the NAS mount from
# record_session_tmux_auto.sh's pane 3), then leave the ros2 bag record of the take-box topic set
# (real hardware) typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.2" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.2" \
  'cd ~/data' C-m
tmux send-keys -t "$SESSION:0.2" \
  'mkdir -p bags && ros2 bag record -o bags/take_box_$(date +%Y%m%d_%H%M%S) /cam_head/cam_head/color/image_raw /cam_left/cam_left/color/image_raw /cam_right/cam_right/color/image_raw /left_joint_trajectory_controller/controller_state /left_joint_trajectory_controller/joint_trajectory /left_revo2_hand_controller/controller_state /left_revo2_hand_controller/joint_trajectory /right_joint_trajectory_controller/controller_state /right_joint_trajectory_controller/joint_trajectory /right_revo2_hand_controller/controller_state /right_revo2_hand_controller/joint_trajectory /tf /tf_static /vr_buttons /left_ee_pose /right_ee_pose'

# Pane 3 (local, not SSH): the VR "connect" client — cd + source install/setup.bash (auto-starts),
# then leave the transrecv_udp client command typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.3" \
  'cd /home/ws/pnk/data_collection/simple_trans_receive/transrecv_udp/ && source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.3" \
  'ros2 run transrecv_udp client 10.87.25.239 1405 2312'

tmux attach -t "$SESSION"
