#!/usr/bin/env bash
# Companion to record_session_tmux_manual.sh — splitting what used to be one 9-pane tmux window
# (which hit tmux's "no space for new pane" error on smaller terminals) into two separate tmux
# sessions you run side by side: this one holds every pane that runs fully automatically with no
# manual Enter needed, in its own 5-pane window:
#
#   pane 0 (ssh JetsonAGX@10.87.25.239, ~/pnk/ws) -> can_configure (auto) + ros2 launch realsense2_camera ... cam_head (auto-run)
#   pane 1 (ssh JetsonAGX@10.87.25.239, ~/pnk/ws) -> ros2 launch realsense2_camera ... cam_left (auto-run)
#   pane 2 (ssh JetsonAGX@10.87.25.239, ~/pnk/ws) -> ros2 launch realsense2_camera ... cam_right (auto-run)
#   pane 3 (local, ~/pnk/ws/dora-openarm-ros2)    -> python3 vr_buttons_episode_logger.py (auto-run)
#   pane 4 (ssh JetsonAGX@10.87.25.239, ~/work/simple_trans_receive) -> transrecv_udp server (auto-run)
#
# See record_session_tmux_manual.sh for the other 4 panes (arm bringup, VR bridge, bag record, VR
# connect client) — those need a manual Enter each, so they live in their own smaller window.
# Pane 0 here (can_configure) should finish before pressing Enter on the manual session's arm
# bringup pane.
#
# SSH login is automated via `sshpass` (requires `sudo apt-get install -y sshpass`) using the
# Jetson's known password, so no manual password entry is needed for the SSH hop itself. There can
# still be a brief connection-establishment delay before the remote shell is ready, though — if a
# pane doesn't show its next command ready right after connecting, just retype it.
#
# SECURITY NOTE: the Jetson's SSH password is embedded in this script (JETSON_PASS below) for
# automation — this repo is pushed to a GitHub remote, so that password is effectively public in
# git history from this commit onward. Prefer switching to SSH key-based auth and dropping
# sshpass/JETSON_PASS entirely if this matters.
#
# Requires tmux (not installed by default on this machine — `sudo apt-get install -y tmux` first).
set -euo pipefail

SESSION="openarm_record_auto"
JETSON="JetsonAGX@10.87.25.232"
JETSON_PASS="1"
SSH_JETSON="sshpass -p '$JETSON_PASS' ssh $JETSON"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists — attaching instead of restarting."
  echo "(Run 'tmux kill-session -t $SESSION' first if you want a clean restart.)"
  tmux attach -t "$SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n auto -c "$HOME/pnk/ws"
tmux split-window -h -t "$SESSION:0" -c "$HOME/pnk/ws"
tmux split-window -v -t "$SESSION:0.0" -c "$HOME/pnk/ws"
tmux split-window -v -t "$SESSION:0.1" -c "$HOME/pnk/ws/dora-openarm-ros2"
tmux split-window -v -t "$SESSION:0.2" -c "$HOME/pnk/ws"
tmux select-layout -t "$SESSION:0" tiled

# Pane 0: SSH into the Jetson, then configure both CAN interfaces, then cam_head (auto-runs) —
# chained on one line so bash itself waits out each sudo prompt before launching the camera,
# instead of racing a second send-keys call against an in-progress password prompt.
tmux send-keys -t "$SESSION:0.0" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.0" \
  'sudo openarm-can-cli -i can0 can_configure && sudo openarm-can-cli -i can1 can_configure && ros2 launch realsense2_camera rs_launch.py camera_name:=cam_head camera_namespace:=cam_head serial_no:=_243222075840 enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x30 initial_reset:=true' C-m

# Pane 1: SSH into the Jetson, then cam_left (auto-runs).
tmux send-keys -t "$SESSION:0.1" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.1" \
  'ros2 launch realsense2_camera rs_launch.py camera_name:=cam_left camera_namespace:=cam_left serial_no:=_260322277300 enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false depth_module.color_profile:=640x480x30' C-m

# Pane 2: SSH into the Jetson, then cam_right (auto-runs).
tmux send-keys -t "$SESSION:0.2" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.2" \
  'ros2 launch realsense2_camera rs_launch.py camera_name:=cam_right camera_namespace:=cam_right serial_no:=_260322270361 enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false depth_module.color_profile:=640x480x30' C-m

# Pane 3: local (no SSH) — auto-runs the VR-buttons episode logger.
tmux send-keys -t "$SESSION:0.3" \
  'python3 vr_buttons_episode_logger.py' C-m

# Pane 4: SSH into the Jetson, cd + source install/setup.bash, then auto-run the transrecv_udp
# server.
tmux send-keys -t "$SESSION:0.4" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.4" \
  'cd ~/work/simple_trans_receive && source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.4" \
  'ros2 run transrecv_udp server 1405 10.87.25.130 2312' C-m

tmux attach -t "$SESSION"
