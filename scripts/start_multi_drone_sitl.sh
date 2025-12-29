#!/bin/bash

PX4_DIR="$HOME/PX4-Autopilot"
PEGASUS_SCRIPT="/home/longhao/source/fsc_PegasusSimulator/application/px4_base/02_px4_multi_drone.py"
ISAAC_PY="$HOME/isaacsim/python_r_fsc.sh"

SESSION="px4_isaac"
BASE_TCP_PORT=4560         # PX4 TCP ports: 4560, 4561, 4562
DELAY=2                    # delay before launching Isaac

# Kill old session if it exists
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

##############################
# Create session + 4 panes
##############################
tmux new-session -d -s "$SESSION" -n "sim"      # window 0, pane 0
tmux split-window -h -t "$SESSION":0           # add pane 1
tmux split-window -h -t "$SESSION":0           # add pane 2
tmux split-window -h -t "$SESSION":0           # add pane 3
tmux select-layout -t "$SESSION":0 even-horizontal

# Now we have 4 panes: 0 1 2 3 (left→right)

##############################
# Pane 0: PX4 uav0 (TCP 4560)
##############################
tmux send-keys -t "$SESSION":0.0 "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [uav_0] (instance 0, TCP 4560)...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_0 \
  ./build/px4_sitl_default/bin/px4 -i 0
echo 'PX4 uav_0 exited.'
exec bash
" C-m


##############################
# Pane 1: PX4 uav1 (TCP 4561)
##############################
tmux send-keys -t "$SESSION":0.1 "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [uav_1] (instance 1, TCP 4561)...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_1 \
  ./build/px4_sitl_default/bin/px4 -i 1
echo 'PX4 uav_1 exited.'
exec bash
" C-m

##############################
# Pane 2: PX4 uav2 (TCP 4562)
##############################
tmux send-keys -t "$SESSION":0.2 "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [uav_2] (instance 2, TCP 4562)...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_2 \
  ./build/px4_sitl_default/bin/px4 -i 2
echo 'PX4 uav_2 exited.'
exec bash
" C-m

##############################
# Pane 3: Isaac Sim + Pegasus
##############################
tmux send-keys -t "$SESSION":0.3 "
echo 'Waiting $DELAY seconds for PX4 instances...'
sleep $DELAY
echo 'Launching Isaac Sim with Pegasus multi-vehicle script...'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
exec bash
" C-m

# Attach to the session (shows the 4 panes)
tmux attach-session -t "$SESSION"
