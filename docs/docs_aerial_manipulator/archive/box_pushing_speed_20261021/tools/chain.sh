#!/bin/bash
# 2026-10-21 push-speed sweep, 240 g box, 0.10 m push, hardware guard values.
# 4 mean speeds x 3 repeats, interleaved. `touch ../PAUSE` holds the chain BEFORE the next
# flight's clean slate (so the planner can be rebuilt between flights); remove it to go on.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_speed_20261021
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
for rep in 1 2 3; do
  for v in 08 13 21 30; do
    while [ -e $C/PAUSE ]; do sleep 5; done
    tag=v${v}_$rep
    clean
    echo "[$(date +%T)] start $tag"
    env MASS=0.240 DIST=0.10 SPEED=0.0$v bash $C/tools/fly.sh $tag $HW
  done
done
clean
echo "[$(date +%T)] CHAIN DONE"
