#!/bin/bash
# 2026-10-27, round 2 -- the room-held descent ("W"): chain.sh's config A (the 10-22 final valve
# setup, the v2 asset, 0.072 N.m stick-slip, 90 deg / 12 s) with push_pull_world_anchor_ready true:
# the claw is held in the ROOM from the end of Go To Start, so the arm -- not the drifting vehicle --
# flies Ready's slide-in. Round 1's drift failures were all the open fingers meeting the fin 11-16 mm
# off radially (sim clearance with the 20 mm grip pad: ~8.75 mm a side). With the free-axis anchor
# (on, latched from the room hold) the claw then rides the base error only along the tangent.
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
    PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
W="push_pull_world_anchor_ready=true"
for k in 2 3 4; do PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs t90W_hw_$k $W; done
gs t90W_nom_1 $W
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
