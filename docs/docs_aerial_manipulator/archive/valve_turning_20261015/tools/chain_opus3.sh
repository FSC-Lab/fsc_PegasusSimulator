#!/bin/bash
# 2026-10-18 (Opus) -- README 15. THE FREE-AXIS ANCHOR: while in CONTACT the claw's reference
# rides the CoM error along the nose (the fin's tangent, the valve's free direction). First the
# build (fsc_autopilot_ros2 + fsc_trajectory_planner -- NOT concurrent with any clean slate: the
# stop scripts pkill by patterns that match their compiler command lines), the parity tests,
# then the shipped turn (omega_c_t 2.0, 36 s) with the anchor at weight 1.0 x3, at 0.5 x1, and
# with omega_c_t 2.927 x2.
set -o pipefail
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
source /opt/ros/humble/setup.bash
source ~/ros2_ws/rosdeps/local_setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
echo "[$(date +%T)] BUILD"
colcon build --packages-select fsc_autopilot_ros2 fsc_trajectory_planner 2>&1 | grep -vE "AMENT_PREFIX_PATH|local_setup" | tail -5
# the whole-body parity suites (law, L1 observer, reference builder): the free-axis anchor is
# default-off, so they must pass unchanged
~/ros2_ws/build/fsc_autopilot_ros2/fsc_autopilot_lib/single_vehicle_baseline/tests/fsc_autopilot_tests \
  --gtest_filter='Wb*' 2>&1 | grep -E "^\[  (PASSED|FAILED)|tests? from .* ran" | sed 's/^/[parity] /'
grep -c "ee_anchor_free_axis_weight" ~/ros2_ws/install/fsc_autopilot_ros2/lib/fsc_autopilot_ros2/autopilot_whole_body_l1_direct_actuation_node | sed 's/^/node has the free-axis key: /'
cd ~
# the abort's motion after the release, variant A: the cut (planned) hold, a 2 s world-held
# release so the arm unfolds off its stops first, then the STRAIGHT climb of the nominal exit
# (variant B, chain 2: back out 0.12 m along the nose, dragged the fin 40 deg)
A="push_pull_abort_back=0.0 push_pull_abort_release_wait_s=2.0"
PL_DRIVER_ARGS="--inject abort@20" run a_abort20 $A
PL_DRIVER_ARGS="--inject abort@33" run a_abort33 $A
PEGASUS_VALVE_FRICTION=0.0 PL_DRIVER_ARGS="--inject abort@20" run a_abort20_f0 $A
for k in 1 2 3; do run fa_w1_$k "wb_ee_anchor_free_axis_weight=1.0"; done
run fa_w05_1 "wb_ee_anchor_free_axis_weight=0.5"
for k in 1 2; do run fa_w1_oct29_$k "wb_ee_anchor_free_axis_weight=1.0" "wb_l1_omega_c_t=2.927"; done
clean
echo "CHAIN_OPUS3 DONE"
