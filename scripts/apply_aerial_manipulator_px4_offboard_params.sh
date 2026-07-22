#!/usr/bin/env bash
# Apply the PX4 parameters required when a wall-clock ROS 2 controller sends
# Offboard setpoints while PX4 itself advances on HIL simulation time.
set -euo pipefail

SESSION="$1"
PANE="$2"
DELAY="${3:-6}"

sleep "$DELAY"

# Without this, the uXRCE-DDS client can re-stamp wall-clock ROS messages into
# PX4 simulation time as stale, preventing Offboard engagement.
tmux send-keys -t "$SESSION:$PANE" "param set UXRCE_DDS_SYNCT 0" Enter
sleep 0.3

# Do not auto-disarm the HIL vehicle during controller startup or while it is
# sitting on the simulated ground. These are per-run changes; do not param save.
tmux send-keys -t "$SESSION:$PANE" "param set COM_DISARM_LAND 0" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param set COM_DISARM_PRFLT 0" Enter
