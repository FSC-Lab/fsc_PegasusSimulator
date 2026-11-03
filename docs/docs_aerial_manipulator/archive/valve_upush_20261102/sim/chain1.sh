#!/bin/bash
# 2026-11-02, the U-push campaign (Pegasus archive/valve_upush_20261102), chain 1: the BARE 5 mm fin
# (PEGASUS_VALVE_FIN_GRIP_MM=0 -- a U never clamps, so it needs no grip shim), the contact point log on.
#   f0: today's half close (the station's Isaac cup 0.012) at today's grasp (25 mm under the fin top)
#   f1: the same, the grasp 20 mm deeper (EE offset z 0.179 -> 0.159): the bracket below the fin top
#   setsid nohup bash chain1.sh > chain1.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
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
  echo "$(date +%H:%M:%S) start $tag gs_extra=${ARM_GS_EXTRA_ARGS:-none} $*"
  PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/$tag.contacts.csv" PEGASUS_VALVE_FIN_GRIP_MM=0 \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
unset PEGASUS_PLANT_FORCE_NOISE
valve f0_bare_cup
valve f1_deep20_cup "push_pull_ee_offset=[0.1600, 0.0006, 0.1590]"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
