#!/bin/bash
# 2026-10-22 -- after chain8 (waits on its PID, $1): repeats of the one survivable drone-mocap
# freeze (s22_final_mfd: the 0.7 s mocap release wait, then straight up), the FINAL config:
#   s22_final_mfd_2      frozen 3 s, 12 s into the turn (s22_final_mfd's case)
#   s22_final_mfd_late   frozen 3 s, 30 s into the turn (the arm's swing-back)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"
export PL_RUNS_DIR="$R"
[[ -n "$1" ]] && while kill -0 "$1" 2>/dev/null; do sleep 20; done
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
FINAL="wb_ee_anchor_free_axis_weight=0.75 wb_ee_anchor_free_axis_on_world=true"
EXTRA="--mocap-fault freeze@push+12:3" gs s22_final_mfd_2 $FINAL
EXTRA="--mocap-fault freeze@push+30:3" gs s22_final_mfd_late $FINAL
clean
echo "CHAIN9 DONE"
