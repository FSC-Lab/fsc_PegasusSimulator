#!/bin/bash
# 2026-11-02, the U-push campaign (Pegasus archive/valve_upush_20261102), chain 2: the BARE 5 mm fin
# (PEGASUS_VALVE_FIN_GRIP_MM=0 -- a U never clamps, so it needs no grip shim), the contact point log on.
#   the DEEP U grasp (archive/valve_upush_20261102 tools/grasp_search.py): the claw point 80 mm under the fin top
#   and 25 mm behind the fin centre -> the bracket's front corner meets the fin's back edge 20-28 mm under its top
#   after ~13 mm; the exit climbs 0.12 m room-held (the claws reach ~0.1 m down the fin)
#   f2 the half close (Isaac cup 0.012) | f3 the jaws fully OPEN (0.019) | f4 the jaws CLOSED (-0.010: the sim claw
#   cannot close below 14 mm, so on the bare 5 mm fin still a U) | f5 70 mm under the top, 20 mm behind
#   setsid nohup bash chain1.sh > chain2.out 2>&1 &      (touch runs/STOP to skip the rest)
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
DEEP80="push_pull_ee_offset=[0.1600, -0.0244, 0.1240]"
EXH="push_pull_exit_world_held_dz=0.12"
valve f2_deep80_cup "$DEEP80" "$EXH"
ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup_position:=0.019" valve f3_deep80_open "$DEEP80" "$EXH"
ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup_position:=-0.010" valve f4_deep80_closed "$DEEP80" "$EXH"
valve f5_deep70_cup "push_pull_ee_offset=[0.1600, -0.0194, 0.1340]" "$EXH"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
