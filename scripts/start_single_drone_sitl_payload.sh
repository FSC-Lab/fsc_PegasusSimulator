#!/bin/bash

PX4_DIR="$HOME/PX4-Autopilot"
PEGASUS_SCRIPT="/home/longhao/source/PegasusSimulator/examples/15_px4_multi_vehicle_ns _slung_load.py"
ISAAC_PY="$HOME/isaacsim/python_r.sh"   # your Isaac launcher
DELAY=2
SESSION="px4_isaac"

# Kill old session if it exists
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# ---- Left pane: PX4 SITL ----
tmux new-session -d -s "$SESSION" -n "sim" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL with DDS namespace uav0...'
PX4_UXRCE_DDS_NS=uav0 make px4_sitl none_iris mavlink_udp_remote:=127.0.0.1 mavlink_udp_port:=14540
echo 'PX4 SITL exited.'
exec bash
"

# ---- Right pane: Isaac Sim + Pegasus ----
tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY sec for PX4...'
sleep $DELAY
echo 'Launching Isaac Sim...'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
exec bash
"

# Make the panes equal width and attach
tmux select-layout -t "$SESSION":0 even-horizontal
tmux attach-session -t "$SESSION"
