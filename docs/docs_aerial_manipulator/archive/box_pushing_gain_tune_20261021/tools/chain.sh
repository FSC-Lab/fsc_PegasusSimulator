#!/bin/bash
# 2026-10-21 box-pushing A/B with the arm GS running Ready To Push -> Push -> Exit by itself
# (station automation, --gs-auto): the free-axis anchor weight 0.75 (the bench's pick,
# ../sweep_r3_hw.txt) vs 0 (today's), 3 flights each, interleaved. The 10-21 hardware task:
# 240 g, 0.10 m at 21 mm/s, hardware guard values. `touch ../PAUSE` holds it before a clean slate.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_gain_tune_20261021
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
R() { [ -e $C/runs/$1.npz ] && { echo "skip $1 (flown)"; return; }; while [ -e $C/PAUSE ]; do sleep 5; done; clean;
      echo "[$(date +%T)] start $1 (free-axis weight $2)";
      env MASS=0.240 DIST=0.10 SPEED=0.021 ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_AUTO_STEPS=true \
        PL_DRIVER_ARGS="--gs-auto" FREE_W=$2 bash $C/tools/fly.sh $1 $HW; }
for rep in 1 2 3; do
  R a0_$rep 0
  R a75_$rep 0.75
done
clean
echo "[$(date +%T)] CHAIN DONE"
