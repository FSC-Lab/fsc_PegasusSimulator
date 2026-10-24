#!/bin/bash
# 2026-10-21 valve LOAD screen (user: a drum on the valve shaft lifting a hanging payload --
# how much resisting torque makes the turn more stable, is it safe?). The shipped valve config
# (README 15.10 of valve_turning_20261015: omega_c_t 2.0, pose [0, 5, 25, 0], guard 10 deg,
# open-then-exit), the measured bearing (0.048 N.m) PLUS a load from the scene's new knobs:
#   PEGASUS_VALVE_LOAD_TORQUE = m g r_drum [N.m], PEGASUS_VALVE_RATCHET=1 = a one-way pawl,
#   PEGASUS_VALVE_DAMPING = a rotary damper [N.m.s/rad].
# Runs land in valve_turning_20261015/runs/ld_* (its campaign_score.py reads them).
#   setsid nohup bash chain.sh [tag:KEY=VAL,KEY=VAL ...] > chain.log 2>&1 &
# With no arguments it flies the first screen below.
V=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1
          ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() {   # run <tag> KEY=VAL,KEY=VAL
  local tag=$1 kv=$2; [ -e $V/runs/$tag.npz ] && { echo "skip $tag (flown)"; return; }
  clean; echo "[$(date +%T)] start $tag ${kv:-baseline}"
  local envs=(); [[ -n "$kv" ]] && IFS=, read -ra envs <<< "$kv"
  env "${envs[@]}" RELEASE_AT_OPEN=0 bash $V/tools/try_valve.sh $tag
  printf '%s\n' "${envs[@]}" >> $V/runs/$tag.args
}
if [[ $# -gt 0 ]]; then
  for a in "$@"; do run "${a%%:*}" "${a#*:}"; done
else
  run ld_w020   PEGASUS_VALVE_LOAD_TORQUE=0.20
  run ld_w020r  PEGASUS_VALVE_LOAD_TORQUE=0.20,PEGASUS_VALVE_RATCHET=1
  run ld_w010r  PEGASUS_VALVE_LOAD_TORQUE=0.10,PEGASUS_VALVE_RATCHET=1
  run ld_w040r  PEGASUS_VALVE_LOAD_TORQUE=0.40,PEGASUS_VALVE_RATCHET=1
  run ld_d050   PEGASUS_VALVE_DAMPING=0.5
fi
clean
echo "[$(date +%T)] CHAIN DONE"
