#!/bin/bash
# 2026-10-18 the turn pose [0, 5, 25, 0] (README 12's best: the same fold q2 + q3 = 30 deg, so the
# claw's pitch and the grasp are unchanged; ~5 deg more margin to the stops on paper). The measured
# bearing, the scheduled turn, jaws closed, the open-then-exit release, no timetable.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
for k in 1 2 3; do run pose525_$k "push_pull_push_pose_deg=[0.0, 5.0, 25.0, 0.0]"; done
clean
echo "CHAIN_POSE DONE"
