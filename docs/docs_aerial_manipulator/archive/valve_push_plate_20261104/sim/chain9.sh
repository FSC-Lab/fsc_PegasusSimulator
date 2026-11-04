#!/bin/bash
# chain 9 (2026-11-04): THE SHIPPED DEFAULTS -- the plate is now the valve design everywhere: the generated
# valve yaml (HANDLE = "plate": the grasp, the four-edge guards, the abort that climbs at once), the scene's
# default asset (Valve_Assembly_v21.usda, fin grip 0, rotor 0.260 kg, bearing 0.072 N.m drive) and the arm
# GS's default claw (CLOSED). NO overrides, no scene knobs, no station arguments:
#   d1 nominal | d2 the bottom edge (the one override: the claw aimed 58.5 mm under the plate top)
#   setsid nohup bash chain9.sh > chain9.out 2>&1 &      (touch runs/STOP to skip the rest)
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
  echo "$(date +%H:%M:%S) start $tag (defaults) $*"
  PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/$tag.contacts.csv" \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
unset PEGASUS_PLANT_FORCE_NOISE PEGASUS_VALVE_FAULT PEGASUS_VALVE_USD PEGASUS_VALVE_FIN_GRIP_MM PEGASUS_VALVE_ROTOR_MASS ARM_GS_EXTRA_ARGS
valve d1_defaults
valve d2_defaults_bottom "push_pull_ee_offset=[0.1600, -0.0613, 0.1455]"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
