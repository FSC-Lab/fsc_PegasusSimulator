#!/bin/bash
# 2026-10-21 the free-axis anchor 0.75 APPLIED (make_push_pull_yaml.py ADDITIONS) + the
# planner's box-lag guard measured against the law's shifted claw target: three flights on
# the SHIPPED sim box yaml (no FREE_W: the yaml carries 0.75), station automation on
# (--gs-auto), the hardware guard values.
#   v75_n1     the 10-21 hardware task: 240 g, 0.10 m at 21 mm/s
#   v75_heavy  the 10-16 worst case h_heavy: 1 kg, mu 0.6, 0.50 m (box lag guard, 8.1 s, then)
#   v75_stop   the 10-16 worst case h_stop: a stopper 0.2 m along the push (box at y 0.28 now,
#              so its +y face at -0.04; 10-16: -0.07 with the box at 0.25), 500 g, 0.50 m
# `touch ../PAUSE` holds it before a clean slate.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_gain_tune_20261021
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
R() { tag=$1; envs=$2; [ -e $C/runs/$tag.npz ] && { echo "skip $tag (flown)"; return; }
      while [ -e $C/PAUSE ]; do sleep 5; done; clean; echo "[$(date +%T)] start $tag ($envs)"
      env ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_AUTO_STEPS=true PL_DRIVER_ARGS="--gs-auto" $envs \
        bash $C/tools/fly.sh $tag $HW; }
R v75_n1    "MASS=0.240 DIST=0.10 SPEED=0.021"
R v75_heavy "MASS=1.000 MU_S=0.6 MU_K=0.6 DIST=0.50"
R v75_stop  "MASS=0.500 DIST=0.50 PEGASUS_PUSH_STOPPER_Y=-0.04"
clean
echo "[$(date +%T)] CHAIN DONE"
