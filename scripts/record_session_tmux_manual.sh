#!/usr/bin/env bash
# Companion to record_session_tmux_auto.sh — splitting what used to be one 9-pane tmux window
# (which hit tmux's "no space for new pane" error on smaller terminals) into two separate tmux
# sessions you run side by side: this one holds every pane whose main command is only typed in,
# waiting for you to press Enter yourself, in its own 5-pane window:
#
#   pane 0 (ssh JetsonAGX@10.87.25.239, ~/pnk/ws) -> ros2 launch openarm_bringup ... (bimanual bringup, typed, NOT auto-run)
#   pane 1 (~/pnk/ws/dora-openarm-ros2, local)    -> venv setup (auto) + uv run dora run ... (VR bridge, typed, NOT auto-run)
#   pane 2 (ssh JetsonAGX@10.87.25.239)           -> ros2 bag record ... (typed, NOT auto-run)
#   pane 3 (local, transrecv_udp)                 -> VR connect client (typed, NOT auto-run)
#   pane 4 (local)                                -> rsync raw_data from the Jetson to this machine (typed, NOT auto-run)
#
# Pane 2's bag record writes to /home/ws/data/real/raw_data/<YYYYMMDD>/<NNN> **on the Jetson
# itself** (NNN = zero-padded 3-digit id, auto-incremented per day). Pane 4 rsyncs that whole
# /home/ws/data/real/raw_data tree from the Jetson to the same path on this machine — run it
# manually (it's typed but not submitted) after stopping a bag recording, whenever you want new
# bags available locally for the bag_to_lerobot.py pipeline. Safe to re-run any time: rsync only
# transfers what's new/changed, so it also works as an incremental sync across a whole day of
# recording.
#
# See record_session_tmux_auto.sh for the other 5 panes (CAN configure + cameras, VR-buttons
# episode logger, transrecv_udp server) — start that script FIRST and let it run for a bit before
# pressing Enter in any of this session's panes. Pane 0 here needs record_session_tmux_auto.sh's
# pane 0 (`can_configure`) to finish first.
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
# the CAN arms are up and all 3 cameras are publishing. This ordering matters: starting the VR
# bridge, the connect client, or the bag recording too early can silently miss real feedback.
#
# Requires tmux (not installed by default on this machine — `sudo apt-get install -y tmux` first).
set -euo pipefail

SESSION="openarm_record_manual"
JETSON="JetsonAGX@10.87.25.232"
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
tmux split-window -v -t "$SESSION:0.2" -c "$HOME"
tmux select-layout -t "$SESSION:0" tiled

# Pane 0: SSH into the Jetson, cd to ~/pnk/ws (auto-starts), then leave the real-hardware bimanual
# bringup typed but NOT submitted (wait for record_session_tmux_auto.sh's CAN configure to finish
# first).
tmux send-keys -t "$SESSION:0.0" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.0" \
  'cd ~/pnk/ws' C-m
tmux send-keys -t "$SESSION:0.0" \
  'ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false right_can_interface:=can0 left_can_interface:=can1 use_fake_hand:=true'

# Pane 1 (local, unchanged): set up the venv (auto-starts), then leave the Dora ROS2<->VR bridge
# command typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.1" \
  'cd ~/pnk/ws/dora-openarm-ros2/ && python3 -m venv .venv && source .venv/bin/activate' C-m
tmux send-keys -t "$SESSION:0.1" \
  'uv run dora run config/dataflow_bridge_ros2_vr_real.yaml --uv'

# Pane 2: SSH into the Jetson (auto-starts), then leave the ros2 bag record of the take-box topic
# set (real hardware) typed but deliberately NOT submitted. Output dir is auto-computed as
# /home/ws/data/real/raw_data/<YYYYMMDD>/<NNN> (NNN = zero-padded 3-digit id, first unused one
# for today), on the Jetson's own filesystem.
tmux send-keys -t "$SESSION:0.2" \
  "$SSH_JETSON" C-m
tmux send-keys -t "$SESSION:0.2" \
  'DAY_DIR=/home/ws/data/real/raw_data/$(date +%Y%m%d) && mkdir -p "$DAY_DIR" && N=0 && while [ -d "$DAY_DIR/$(printf %03d $N)" ]; do N=$((N+1)); done && OUT_DIR="$DAY_DIR/$(printf %03d $N)" && ros2 bag record -o "$OUT_DIR" /cam_head/cam_head/color/image_raw /cam_left/cam_left/color/image_raw /cam_right/cam_right/color/image_raw /left_joint_trajectory_controller/controller_state /left_joint_trajectory_controller/joint_trajectory /left_revo2_hand_controller/controller_state /left_revo2_hand_controller/joint_trajectory /right_joint_trajectory_controller/controller_state /right_joint_trajectory_controller/joint_trajectory /right_revo2_hand_controller/controller_state /right_revo2_hand_controller/joint_trajectory /tf /tf_static /vr_buttons /left_ee_pose /right_ee_pose'

# Pane 3 (local, not SSH): the VR "connect" client — cd + source install/setup.bash (auto-starts),
# then leave the transrecv_udp client command typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.3" \
  'cd /home/ws/pnk/data_collection/simple_trans_receive/ && source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.3" \
  'ros2 run transrecv_udp client 10.87.25.232 1405 2312'

# Pane 4 (local, not SSH): rsync the Jetson's real raw_data tree onto this machine — typed but
# deliberately NOT submitted, run it yourself whenever you want newly-recorded bags available
# locally. sshpass reuses the same Jetson password as the SSH panes above (rsync invokes its own
# ssh transport, separate from $SSH_JETSON's interactive shell).
tmux send-keys -t "$SESSION:0.4" \
  "mkdir -p /home/ws/data/real/raw_data && sshpass -p '$JETSON_PASS' rsync -avz --progress -e ssh $JETSON:/home/ws/data/real/raw_data/ /home/ws/data/real/raw_data/"

tmux attach -t "$SESSION"
