#!/bin/bash
# 2026-10-21 release-delay A/B (fix 1 of the 10-21 flight report): the arm GS's OWN automatic
# exit after the push, 0.2 s (new default) vs 1.0 s (the flown value), 3 flights each,
# interleaved, the 10-21 hardware task: 240 g, 0.10 m at 21 mm/s, hardware guard values.
# `touch ../PAUSE` holds it before the next flight's clean slate.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_release_delay_20261021
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
R() { [ -e $C/runs/$1.npz ] && { echo "skip $1 (flown)"; return; }; while [ -e $C/PAUSE ]; do sleep 5; done; clean; echo "[$(date +%T)] start $1 (GS delay $2 s)";
      env MASS=0.240 DIST=0.10 SPEED=0.021 ARM_GS_AUTO_EXIT_DELAY=$2 PL_DRIVER_ARGS="--gs-exit" \
        bash $C/tools/fly.sh $1 $HW; }
for rep in 1 2 3; do
  R d02_$rep 0.2
  R d10_$rep 1.0
done
clean
echo "[$(date +%T)] CHAIN DONE"
