#!/bin/bash
# 2026-10-28 night: the EXIT FIX (planner push_pull_exit_world_held_dz 0.06: the open claw stays
# room-held for the first 60 mm of the climb) + the 4 deg rotor-tilt limit, from the regenerated
# valve yaml. The exit-failure seed (2) twice, then 3 and 4. Hardware-like drift (0.32 N, 0.12 Hz),
# v2 valve at 0.072 N.m (drive), GS-run steps, each run stops 4 s after the exit (chain.sh's harness).
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
for run in s2a:2 s2b:2 s3:3 s4:4; do k=${run#*:}; t=${run%%:*}
  export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k"
  gs ex_$t
done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
