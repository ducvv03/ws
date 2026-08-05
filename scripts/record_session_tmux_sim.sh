#!/usr/bin/env bash
# Sim-mode counterpart to record_session_tmux.sh — opens a 4-terminal OpenArm Revo2
# VR-bridge/recording session as one tmux window with 4 tiled panes:
#
#   pane 0 (~/pnk/ws/dora-openarm-ros2) -> venv setup (auto) + uv run dora run ... _sim.yaml (VR bridge, typed, NOT auto-run)
#   pane 1                              -> ros2 bag record ... (typed, NOT auto-run)
#   pane 2                              -> conda activate sim + isaacsim (auto-start)
#   pane 3 (~/pnk/ws/dora-openarm-ros2) -> python3 vr_buttons_episode_logger.py (auto-start)
#
# No fake-hardware bringup pane here — bring the arm up separately if/when you need it running.
#
# Pane 0 (VR bridge) and pane 1 (rosbag record) are typed but NOT submitted — press Enter in each
# only after confirming that the arm (however you brought it up) is active. This ordering matters:
# starting the VR bridge or the bag recording too early can silently miss feedback. Panes 2
# (Isaac Sim) and 3 (VR-buttons episode logger) auto-start right away since they're independent
# of the arm/VR bridge readiness.
#
# NOTE: --mode sim on the dora-to-ros2 node publishes a single merged /joint_command
# (sensor_msgs/JointState) instead of the real-mode per-arm/per-hand JointTrajectory command
# topics, and there is no per-controller controller_state feedback to record either — use the
# aggregate /joint_states topic instead (see the bag command below).
#
# Requires tmux (not installed by default on this machine — `sudo apt-get install -y tmux` first).
set -euo pipefail

SESSION="openarm_record_sim"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists — attaching instead of restarting."
  echo "(Run 'tmux kill-session -t $SESSION' first if you want a clean restart.)"
  tmux attach -t "$SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n record_sim -c "$HOME/pnk/ws/dora-openarm-ros2"
tmux split-window -h -t "$SESSION:0" -c "$HOME/data"
tmux split-window -v -t "$SESSION:0.1"
tmux split-window -v -t "$SESSION:0.0" -c "$HOME/pnk/ws/dora-openarm-ros2"
tmux select-layout -t "$SESSION:0" tiled

# Pane 0: set up the venv (auto-starts), then leave the Dora ROS2<->VR bridge (sim dataflow)
# command typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.0" \
  'cd ~/pnk/ws/dora-openarm-ros2/ && python3 -m venv .venv && source .venv/bin/activate' C-m
tmux send-keys -t "$SESSION:0.0" \
  'uv run dora run config/dataflow_bridge_ros2_vr_sim.yaml --uv'

# Pane 1: leave the ros2 bag record of the take-box topic set (sim) typed but deliberately NOT
# submitted. Output dir is auto-computed as /home/ws/data/sim/raw_data/<YYYYMMDD>/<NNN> (NNN =
# zero-padded 3-digit id, first unused one for today).
tmux send-keys -t "$SESSION:0.1" \
  'DAY_DIR=/home/ws/data/sim/raw_data/$(date +%Y%m%d) && mkdir -p "$DAY_DIR" && N=0 && while [ -d "$DAY_DIR/$(printf %03d $N)" ]; do N=$((N+1)); done && OUT_DIR="$DAY_DIR/$(printf %03d $N)" && ros2 bag record -o "$OUT_DIR" /cam_head/cam_head/color/image_raw /cam_left/cam_left/color/image_raw /cam_right/cam_right/color/image_raw /tf /tf_static /vr_buttons /left_ee_pose /right_ee_pose /joint_command /joint_states'

# Pane 2: activate the sim conda env and launch Isaac Sim (auto-starts).
tmux send-keys -t "$SESSION:0.2" \
  'conda activate sim && isaacsim' C-m

# Pane 3: VR-buttons episode logger (auto-starts).
tmux send-keys -t "$SESSION:0.3" \
  'python3 vr_buttons_episode_logger.py' C-m

tmux attach -t "$SESSION"
