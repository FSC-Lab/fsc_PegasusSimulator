#!/bin/bash
# 2026-10-18 valve tuning, phase 7: THE EXIT. omega_c_t 2.0 completed 3 of 3 in tuning but 2 of 7 on
# the results timetable -- every loss the exit's wrist roll: with --release-at-open the claw is
# handed to the CoM anchor (1 s blend) BEFORE the jaws open, so the open jaws follow the base's
# 20-50 mm wander, one presses the fin (which cannot move radially) ~18 mm off the wrist axis,
# and the soft heading lets q4 roll 30-45 deg (the law's own j4 torque OPPOSES it). Fix candidate:
# open with the claw still WORLD-held (RELEASE_AT_OPEN=0: open, wait, Exit) and hand over SLOWLY
# (wb_ee_anchor_blend_s 3.0) while the exit climbs the 25 mm off the fin.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
while pgrep -f "chain_ca1.sh" > /dev/null; do sleep 10; done      # after the comparison round
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; rel=$2; shift 2; clean; RELEASE_AT_OPEN=$rel PEGASUS_VALVE_FRICTION=0.048 bash $C/tools/try_valve.sh $tag "$@"; }
TT="--timetable --slot-release 39.0 --slot-land-step 28.0"
export PL_DRIVER_ARGS="$TT"
run i_old_b3    0 wb_ee_anchor_blend_s=3.0
run i_old_b3_r2 0 wb_ee_anchor_blend_s=3.0
run i_rel_b3    1 wb_ee_anchor_blend_s=3.0
run i_old_b1    0
clean
echo CHAIN_I DONE
