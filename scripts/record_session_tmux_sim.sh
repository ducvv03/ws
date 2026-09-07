#!/usr/bin/env bash
# Sim-mode counterpart to record_session_tmux.sh — opens a 3-terminal OpenArm Revo2
# fake-hardware session as one tmux window with 3 tiled panes:
#
#   pane 0 (~/pnk/ws)                   -> ros2 launch pnk_bringup ... use_fake_hardware:=true (auto-start)
#   pane 1 (~/pnk/ws/dora-openarm-ros2) -> venv setup (auto) + uv run dora run ... _sim.yaml (VR bridge, typed, NOT auto-run)
#   pane 2 (~/pnk/ws)                   -> ros2 bag record ... (typed, NOT auto-run)
#
# No CAN configuration and no cameras here — fake hardware needs neither. Pane 0 is safe to
# auto-start since it never touches real CAN/motors.
#
# Pane 1 (VR bridge) and pane 2 (rosbag record) are typed but NOT submitted — press Enter in each
# only after confirming (from pane 0's logs) that the fake-hardware controllers are active. This
# ordering matters: starting the VR bridge or the bag recording too early can silently miss
# feedback.
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

tmux new-session -d -s "$SESSION" -n record_sim -c "$HOME/pnk/ws"
tmux split-window -h -t "$SESSION:0" -c "$HOME/pnk/ws/dora-openarm-ros2"
tmux split-window -v -t "$SESSION:0.1" -c "$HOME/pnk/ws"
tmux select-layout -t "$SESSION:0" tiled

# Pane 0: fake-hardware bimanual bringup (auto-starts — no real CAN/motors involved).
tmux send-keys -t "$SESSION:0.0" \
  'ros2 launch pnk_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true use_fake_hand:=true' C-m

# Pane 1: set up the venv (auto-starts), then leave the Dora ROS2<->VR bridge (sim dataflow)
# command typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.1" \
  'cd ~/pnk/ws/dora-openarm-ros2/ && python3 -m venv .venv && source .venv/bin/activate' C-m
tmux send-keys -t "$SESSION:0.1" \
  'uv run dora run config/dataflow_bridge_ros2_vr_sim.yaml --uv'

# Pane 2: ros2 bag record of the take-box topic set (sim) — typed but deliberately NOT submitted.
tmux send-keys -t "$SESSION:0.2" \
  'mkdir -p bags && ros2 bag record -o bags/take_box_sim_$(date +%Y%m%d_%H%M%S) /cam_head/cam_head/color/image_raw /cam_left/cam_left/color/image_raw /cam_right/cam_right/color/image_raw /tf /tf_static /vr_buttons /left_ee_pose /right_ee_pose /joint_command /joint_states'

tmux attach -t "$SESSION"
