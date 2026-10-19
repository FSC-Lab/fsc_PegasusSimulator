#!/bin/bash
# 2026-10-09 A/B: push vs pull at the SHIPPED push yaml (3 + 3), on the driver's timetable.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_tune_20261009
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
for k in 1 2 3; do
  clean; bash $C/tools/fly.sh ab_push_$k
  clean; PULL=1 bash $C/tools/fly.sh ab_pull_$k
done
clean
echo CHAIN_AB DONE
