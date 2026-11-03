#!/bin/bash
# 2026-11-03, the fin-loss safety pass (Pegasus archive/valve_fin_guard_20261103): the U push with the planner's NEW
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
GATES=("push_pull_ready_max_speed=0.06" "push_pull_handle_settle_tol=0.035" "push_pull_handle_settle_max_speed=0.05")
GSGATES="-p push_pull_valve_grip_axis_tol:=0.008 -p push_pull_valve_settle_wait_max_s:=12.0 -p push_pull_valve_grip_wait_max_s:=5.0"
unset PEGASUS_PLANT_FORCE_NOISE PEGASUS_VALVE_FAULT FRICTION GS
valve g1_nominal
PEGASUS_VALVE_FAULT=force,30,1.5,0,0,15 valve g2_lift
PEGASUS_VALVE_FAULT=force,30,1.5,-10,0,0 valve g3_back
PEGASUS_VALVE_FAULT=force,30,1.5,0,10,0 valve g4_side
PEGASUS_VALVE_FAULT=jam,30 valve g5_jam
PEGASUS_VALVE_FAULT=drop,30,1.0 valve g6_drop
PEGASUS_VALVE_FAULT=freeze,30,1.0 valve g7_freeze
FRICTION=0.0 valve g8_frictionless
for k in 2 3 4; do PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,$k GS="$GSGATES" valve g$((7 + k))_drift$k "${GATES[@]}"; done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
