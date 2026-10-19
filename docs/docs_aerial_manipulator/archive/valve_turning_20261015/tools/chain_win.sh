#!/bin/bash
# 2026-10-18 ONE WINDOWED valve flight (the scene with its window on DISPLAY :1) at the shipped
# configuration, to measure the real-time factor per phase under the real load with 06's
# slack-gated viewport rendering (PEGASUS_RENDER_ADAPTIVE, default 1). Run with setsid nohup.
#   chain_win.sh <tag> [PEGASUS_RENDER_ADAPTIVE=0|1]
# After the flight the Isaac pane (the pacer's "RTF x.xxx ... fps" lines) is captured from every
# tmux pane into runs/<tag>.isaac_pane.log BEFORE the clean slate kills the server.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
TAG="${1:?tag}"; ADAPT="${2:-1}"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
clean
VALVE_WINDOWED=1 PEGASUS_RENDER_ADAPTIVE=$ADAPT RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh "$TAG"
# the Isaac pane's pacer lines (any tmux session/pane that printed them)
for p in $(tmux list-panes -a -F '#S:#I.#P' 2>/dev/null); do
  tmux capture-pane -p -J -S -20000 -t "$p" 2>/dev/null | grep -E "RTF|PACER|RENDER" | sed "s|^|[$p] |"
done > "$C/runs/$TAG.isaac_pane.log"
echo "captured $(wc -l < "$C/runs/$TAG.isaac_pane.log") pacer lines"
clean
echo "CHAIN_WIN DONE"
