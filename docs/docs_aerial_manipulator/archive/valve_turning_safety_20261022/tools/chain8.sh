#!/bin/bash
# 2026-10-22 -- the candidate FINAL valve configuration. chain7 (the measured-hold / unbuckle
# A/B) is stopped with runs/STOP once its mocap-freeze runs are flown. The A/B's verdict: the
# unbuckle abort crashed the injected mid-turn abort (s22_um_ab20, arm on both stops) as the
# measured hold did on 10-18, so the abort stays the shipped PLANNED hold.
#  0. clean; build the planner (the mocap-abort fix + the LIVE-HANDLE close gate); loopback
#  1. FINAL = the shipped valve config + the box push's free axis (0.75, latched from the room
#     hold), the drift model:  s22_final_hw_2/3/4 (seeds 2, 3 grasped the fin's corner and
#     flipped on the shipped config -- the gate should refuse that close)
#  2. s22_final_nom_1/2       FINAL without the drift model
#  3. s22_final_mfd           FINAL, the drone's mocap frozen 3 s at 12 s
#  4. s22_final_gov_hv        FINAL + push_pull_follow, the heavy valve (damping 2.0)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"
export PL_RUNS_DIR="$R"
CH7=$(pgrep -f "bash tools/chain7.sh" | head -1)
if [[ -n "$CH7" ]]; then
  until grep -q "^\[s22_mfd_fix_1\]" "$R/chain7.log" 2>/dev/null || ! kill -0 "$CH7" 2>/dev/null; do sleep 15; done
  touch "$R/STOP"
  while kill -0 "$CH7" 2>/dev/null; do sleep 10; done
fi
rm -f "$R/STOP"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
clean
echo "BUILD $(date +%T)"
( cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && \
  nice -n 5 colcon build --symlink-install --packages-select fsc_trajectory_planner ) > "$R/chain8_build.log" 2>&1 \
  && echo "BUILD OK $(date +%T)" || { echo "BUILD FAILED -- see chain8_build.log"; exit 1; }
grep -qaF "THE LIVE HANDLE" ~/ros2_ws/src/fsc_trajectory_planner/src/whole_body_trajectory_planner_node.cpp && echo "source has the live handle"
echo "LOOPBACK $(date +%T)"
( source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && \
  ROS_DOMAIN_ID=78 timeout 900 python3 ~/ros2_ws/src/fsc_trajectory_planner/test/test_push_pull_loopback.py ) \
  > "$R/chain8_loopback.log" 2>&1
grep -E "ALL PASS|Error|assert" "$R/chain8_loopback.log" | head -5
FINAL="wb_ee_anchor_free_axis_weight=0.75 wb_ee_anchor_free_axis_on_world=true"
for k in 2 3 4; do PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs s22_final_hw_$k $FINAL; done
for k in 1 2; do gs s22_final_nom_$k $FINAL; done
EXTRA="--mocap-fault freeze@push+12:3" gs s22_final_mfd $FINAL
PEGASUS_VALVE_DAMPING=2.0 gs s22_final_gov_hv $FINAL push_pull_follow=true
clean
echo "CHAIN8 DONE"
