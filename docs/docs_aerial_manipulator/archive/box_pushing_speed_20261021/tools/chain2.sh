#!/bin/bash
# 2026-10-21, after chain.sh was stopped at its pause (v08_1, v13_1 valid; the first v21_1
# INVALID -- it flew the old planner's built-in 12 s): the rest of the sweep on the NEW planner
# (push_pull_push_mean_speed sets the time; fly.sh gives it the SPEED and no floor).
# Same pause flag: `touch ../PAUSE` holds it before the next flight's clean slate.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_speed_20261021
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
R() { while [ -e $C/PAUSE ]; do sleep 5; done; clean; echo "[$(date +%T)] start $1";
      env MASS=0.240 DIST=0.10 SPEED=0.0$2 bash $C/tools/fly.sh $1 $HW; }
R v21_1 21
R v30_1 30
for rep in 2 3; do
  for v in 08 13 21 30; do R v${v}_$rep $v; done
done
clean
echo "[$(date +%T)] CHAIN2 DONE"
