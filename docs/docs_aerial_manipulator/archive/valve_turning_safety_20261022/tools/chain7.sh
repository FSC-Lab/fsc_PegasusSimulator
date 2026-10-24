#!/bin/bash
# 2026-10-22 -- after chain6 (waits on its PID, $1; chain6 was stopped with runs/STOP after
# s22_fa_hw_3, this removes it). s22_fa_hw_2 never grasped: with wb_ee_anchor_free_axis_on_world
# the free axis acts during the OPEN-jaw wait on the handle too, and the open claw rode the drift
# 31-54 mm along the nose -- off the fin's edge -- and closed on nothing. On the valve the shipped
# config's drift failures all began AFTER the grasp, so the free axis latched at the grasp only
# (on_world false) is flown first:
#   s22_fa0_hw_1..3   free axis 0.75, on_world FALSE, measured hold + unbuckle, drift seeds 1..3
# then chain6's remaining list.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PL_RUNS_DIR="$HERE/../runs"
[[ -n "$1" ]] && while kill -0 "$1" 2>/dev/null; do sleep 20; done
rm -f "$HERE/../runs/STOP"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$HERE/../runs/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
UM="push_pull_abort_from_measured=true push_pull_abort_unbuckle=true"
FA0="wb_ee_anchor_free_axis_weight=0.75 wb_ee_anchor_free_axis_on_world=false $UM"
for k in 2 1 3; do PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs s22_fa0_hw_$k $FA0; done
for k in 1 2; do PEGASUS_VALVE_DAMPING=2.0 gs s22_um_hv_$k $UM; done
EXTRA="--inject abort@20" gs s22_um_ab20 $UM
EXTRA="--inject abort@33" gs s22_um_ab33 $UM
EXTRA="--mocap-fault freeze@push+12:3" gs s22_um_mfd $UM
EXTRA="--mocap-fault freeze@push+12:3" gs s22_mfd_fix_1
for k in 1 2; do gs s22_fa0_nom_$k $FA0; done
for k in 1 2; do gs s22_m250_$k sim_feedback_mocap_rate_hz=250.0; done
clean
echo "CHAIN7 DONE"
