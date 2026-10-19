#!/bin/bash
# 2026-10-18 PATH FOLLOWING for the turn (README 11): the governed-progress turn
# (push_pull_follow) against the scheduled one, nominal / heavy / stuck valve, headless and
# windowed. Every run: the open-with-the-claw-world-held exit (RELEASE_AT_OPEN=0), no timetable.
#   chain_pf.sh <phase>   (one | rest)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; fric=$2; win=$3; shift 3; clean; VALVE_WINDOWED=$win RELEASE_AT_OPEN=0 PEGASUS_VALVE_FRICTION=$fric bash $C/tools/try_valve.sh $tag "$@"; }
case "${1:-one}" in
  one)
    run pf_n1 0.048 0 push_pull_follow=true ;;
  rest)
    # (pf_n1 flew the first governor, unfiltered channels; these fly the low-passed one)
    run pf_stuck   1.0   0 push_pull_follow=true
    run base_stuck 1.0   0
    run pf_n2      0.048 0 push_pull_follow=true
    run pf_n3      0.048 0 push_pull_follow=true
    run base_n1    0.048 0
    run pf_n4      0.048 0 push_pull_follow=true
    run pf_win1    0.048 1 push_pull_follow=true
    run pf_win2    0.048 1 push_pull_follow=true ;;
esac
clean
echo "CHAIN_PF ${1:-one} DONE"
