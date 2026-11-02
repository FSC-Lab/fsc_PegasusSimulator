#!/bin/bash
# 2026-11-02, the abort on the SHIPPED valve yaml (Pegasus archive/valve_cup_abort_20261102): after chain_ab.sh. The
# planner reads the gripper's own joint state (push_pull_abort_gripper_*), climbs once it reads open (at most 1 s), the
# first 60 mm in 0.8 s (push_pull_abort_fast_climb), the claw following the vehicle; the half close on the handle.
#   setsid nohup bash chain_final.sh > chain_final.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
until grep -q "CHAIN DONE" "$HERE/chain_ab.out" 2>/dev/null; do sleep 10; done
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }
V=$A/valve_turning_20261015
DRV=$A/valve_box_isolation_20261102/tools/driver
valve() {
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag hold=${HOLD:-0} $* ${EXTRA:-}"
  local hold_args=""; [[ -n "${HOLD:-}" ]] && hold_args="--gs-hold $HOLD"
  PL_RUNS_DIR="$R" \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S="${HOLD:-0.0}" RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 $hold_args ${EXTRA:-}" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
unset PEGASUS_PLANT_FORCE_NOISE
# the operator's ABORT in a half-close static hold, 3 s after the grasp
HOLD=10.0 EXTRA="--inject abort@3" valve fin_abort_hold
# the operator's ABORT while the U pushes the fin, 10 s into the turn
EXTRA="--inject abort@10" valve fin_abort_turn
# a GUARD abort mid-turn: the q3 floor raised to 22 deg so the static q3 guard trips in the turn
valve fin_q3trip push_pull_guard_q3_min_deg=22.0
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
