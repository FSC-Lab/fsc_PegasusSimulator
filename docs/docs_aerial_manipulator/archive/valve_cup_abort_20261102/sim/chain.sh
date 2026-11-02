#!/bin/bash
# 2026-11-02, the valve's HALF CLOSE and the climb-on-open ABORT (Pegasus archive/valve_cup_abort_20261102), in
# Isaac: the v2 valve at 0.072 N.m (drive model), raw mocap, headless, the ARM GS flying every step by itself
# (--gs-auto, as on hardware), each run stopped 4 s after the exit or the abort. The station's half close in
# Isaac is its simulation default (push_pull_valve_cup_position 0.012 m); the valve yaml carries the abort keys
# (push_pull_abort_climb_on_open, _open_timeout_s 3.0, _fast_follow, _exit_hold false).
#   setsid nohup bash chain.sh [job ...] > chain.out 2>&1 &      (touch runs/STOP to skip the rest)
# Jobs: hold turn aborts drift (default: all four)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }

V=$A/valve_turning_20261015
DRV=$A/valve_box_isolation_20261102/tools/driver   # the isolation's driver (the grasp event, --gs-hold)
valve() {   # one valve flight, the GS flying every step; EXTRA = more driver args, HOLD = the static hold [s]
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag hold=${HOLD:-0} noise=${PEGASUS_PLANT_FORCE_NOISE:-none} $* ${EXTRA:-}"
  local hold_args=""; [[ -n "${HOLD:-}" ]] && hold_args="--gs-hold $HOLD"
  PL_RUNS_DIR="$R" \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S="${HOLD:-0.0}" RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 $hold_args ${EXTRA:-}" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}

run() {
  case $1 in
    hold)     # the half close in a static hold: does it touch the fin? the exit after it
      unset PEGASUS_PLANT_FORCE_NOISE
      HOLD=10.0 valve cup_hold_1 ;;
    turn)     # the 90 deg turn with the half close: does the U drive the valve?
      unset PEGASUS_PLANT_FORCE_NOISE
      valve cup_turn_1
      valve cup_turn_2 ;;
    aborts)   # the abort: the jaws open at once, the climb on their OPEN report, the claw following the vehicle
      unset PEGASUS_PLANT_FORCE_NOISE
      HOLD=10.0 EXTRA="--inject abort@3" valve cup_abort_hold
      EXTRA="--inject abort@6" valve cup_abort_turn
      # a GUARD abort mid-turn: the sway guard at 0.05 m/s on drift seed 2
      PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2" valve cup_swaytrip push_pull_guard_sway_speed=0.05 ;;
    drift)    # the turn under the hardware-like drift
      PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2" valve cup_turn_s2
      PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,3" valve cup_turn_s3 ;;
    *) echo "unknown job $1" ;;
  esac
}
for j in "${@:-hold turn aborts drift}"; do for k in $j; do run "$k"; done; done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
