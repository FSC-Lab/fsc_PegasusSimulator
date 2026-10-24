#!/bin/bash
# 2026-10-22 -- re-ordered after chain1 (the shipped valve config failed the box push's
# hardware-like drift model 0 of 3, s22_hw_1..3). Replaces chain2..5 (stopped before they flew):
#  0. clean; build the planner with the mocap-abort fix (no stack up); its loopback
#  1. s22_fa_hw_1..3   the box push's fix: free axis 0.75 + on_world, measured hold + unbuckle,
#                      drift seeds 1..3 (= s22_hw_1..3's)
#  2. s22_um_hv_1..2   heavy valve, measured hold + unbuckle (s22_heavy flipped on the planned hold)
#     s22_um_ab20/33   injected aborts at 20 / 33 s, measured hold + unbuckle
#     s22_um_mfd       drone mocap frozen 3 s at 12 s, measured hold + unbuckle (+ the fix)
#     s22_mfd_fix_1    the same on the shipped planned hold (+ the fix)
#  3. s22_fa_nom_1..2  free axis + unbuckle, no drift (does it cost the nominal turn?)
#     s22_m250_1..2    shipped, mocap 250 Hz (every 10-18 flight's rate)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PL_RUNS_DIR="$HERE/../runs"
[[ -n "$1" ]] && while kill -0 "$1" 2>/dev/null; do sleep 20; done
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$HERE/../runs/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
clean
echo "BUILD $(date +%T)"
( cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && \
  nice -n 5 colcon build --symlink-install --packages-select fsc_trajectory_planner ) > "$HERE/../runs/chain6_build.log" 2>&1 \
  && echo "BUILD OK $(date +%T)" || { echo "BUILD FAILED -- see chain6_build.log"; exit 1; }
echo "LOOPBACK $(date +%T)"
( source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && \
  ROS_DOMAIN_ID=78 timeout 900 python3 ~/ros2_ws/src/fsc_trajectory_planner/test/test_push_pull_loopback.py ) \
  > "$HERE/../runs/chain6_loopback.log" 2>&1
grep -E "^\[9d\]|^\[9j\]|ALL PASS|Error|assert" "$HERE/../runs/chain6_loopback.log" | head -12
UM="push_pull_abort_from_measured=true push_pull_abort_unbuckle=true"
FA="wb_ee_anchor_free_axis_weight=0.75 wb_ee_anchor_free_axis_on_world=true $UM"
for k in 1 2 3; do PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs s22_fa_hw_$k $FA; done
for k in 1 2; do PEGASUS_VALVE_DAMPING=2.0 gs s22_um_hv_$k $UM; done
EXTRA="--inject abort@20" gs s22_um_ab20 $UM
EXTRA="--inject abort@33" gs s22_um_ab33 $UM
EXTRA="--mocap-fault freeze@push+12:3" gs s22_um_mfd $UM
EXTRA="--mocap-fault freeze@push+12:3" gs s22_mfd_fix_1
for k in 1 2; do gs s22_fa_nom_$k $FA; done
for k in 1 2; do gs s22_m250_$k sim_feedback_mocap_rate_hz=250.0; done
clean
echo "CHAIN6 DONE"
