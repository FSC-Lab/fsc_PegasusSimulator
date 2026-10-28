#!/bin/bash
# 2026-10-27, round 3 -- the HARDWARE-PREP config as generated (no overrides): W (the room-held
# descent) + the keep-outs placed from the valve + the box-style way home (Land Start over the
# start mark at 1.3 m, Land to 0.30 m) + 90 deg / 12 s. Two FULL missions (the landing changed)
# and the drift seeds 2/3/4 to the exit. The v2 asset at its measured 0.072 N.m stick-slip.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; echo "$(date +%H:%M:%S) start $tag"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
gs hwp_full_1
for k in 2 3 4; do EXTRA="--stop-after-exit --stop-hold 4" PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs hwp_hw_$k; done
gs hwp_full_2
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
