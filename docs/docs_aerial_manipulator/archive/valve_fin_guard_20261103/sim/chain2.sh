#!/bin/bash
# (chain 2: see the bottom) 2026-11-03, the fin-loss safety pass (Pegasus archive/valve_fin_guard_20261103): the U push with the planner's NEW
# fin-in-the-U monitor + clearance guard, the 0.2 s feed timeout, the 2 s governor hold and the one-climb abort (all in
# the GENERATED valve yaml -- try_valve.sh regenerates it), the bare fin, the station flying every step after Go To
# Start. Faults from the scene's PEGASUS_VALVE_FAULT, 30 deg into the turn:
#   g1 nominal | g2 lift +15 N 1.5 s | g3 back -10 N 1.5 s | g4 side +10 N (toward the axis) 1.5 s | g5 jam |
#   g6 the fin's mocap dropped 1 s | g7 frozen 1 s | g8 a frictionless valve | g9-g11 drift seeds 2/3/4 (the
#   approach gates relaxed as in the U-push campaign's chain 4/5: the shipped ones abort before contact under drift)
#   setsid nohup bash chain1.sh > chain1.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }
V=$A/valve_turning_20261015
DRV=$A/valve_upush_20261102/tools/driver
valve() {
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag fault=${PEGASUS_VALVE_FAULT:-none} friction=${FRICTION:-0.072} noise=${PEGASUS_PLANT_FORCE_NOISE:-none} gs=${GS:-} $*"
  PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/$tag.contacts.csv" PEGASUS_VALVE_FIN_GRIP_MM=0 \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=${FRICTION:-0.072} PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
  ARM_GS_EXTRA_ARGS="${GS:-}" \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
# chain 2 (2026-11-03): the same three forces at a REALISTIC size -- chain 1's 10-15 N for 1.5 s (27-40 % of the
# 36.7 N weight, still pushing through the abort) buckled the arm in 0.1-0.4 s and two of the three crashed
until grep -q "CHAIN DONE" "$HERE/chain1.out" 2>/dev/null; do sleep 15; done
unset PEGASUS_PLANT_FORCE_NOISE PEGASUS_VALVE_FAULT FRICTION GS
PEGASUS_VALVE_FAULT=force,30,1.0,-3,0,0 valve h1_back3
PEGASUS_VALVE_FAULT=force,30,1.0,0,3,0 valve h2_side3
PEGASUS_VALVE_FAULT=force,30,1.0,0,0,5 valve h3_lift5
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
