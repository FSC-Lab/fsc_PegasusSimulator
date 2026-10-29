#!/bin/bash
# 2026-10-28 evening: Isaac safety/trade screen after the 10-28 hardware valve flights. The flown law (base),
# a damping raise on the CoM loop (wb_k_v 12.58 -> 15, the hardware's 0.58 Hz contact mode), and a lower
# free-axis weight (0.75 -> 0.5, valve-angle accuracy vs the arm's share of the wander). Hardware-like drift
# seeds 2-4 (PEGASUS_PLANT_FORCE_NOISE 0.32 N, 0.12 Hz), v2 valve at 0.072 N.m (drive), GS-run steps, each
# run stops 4 s after the exit. Harness: valve_turning_20261015/tools/try_valve.sh (10-27 chain4.sh pattern).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/runs"; mkdir -p "$R"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; echo "$(date +%H:%M:%S) start $tag $*"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" bash $C/tools/try_valve.sh "$tag" "$@"; }
for k in 2 3 4; do
  export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k"
  gs base_s$k
  gs kv15_s$k wb_k_v=15.0
  gs fa05_s$k wb_ee_anchor_free_axis_weight=0.5
done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
