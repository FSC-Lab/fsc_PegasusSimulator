#!/bin/bash
# 2026-10-16 box-pushing campaign, stages 0 + 1: the baseline (400 g, 0.5 m, the 10-09
# push yaml with the heading pinned) repeated 3x on the moved table + Vertical Margin 0.15,
# and the mass screen 250 / 500 / 600 g, 2 runs each, interleaved.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; env "$@" bash $C/tools/fly.sh $tag; }
run s0_base_1 MASS=0.400
run s1_m250_1 MASS=0.250
run s1_m600_1 MASS=0.600
run s1_m500_1 MASS=0.500
run s0_base_2 MASS=0.400
run s1_m250_2 MASS=0.250
run s1_m600_2 MASS=0.600
run s1_m500_2 MASS=0.500
run s0_base_3 MASS=0.400
clean
echo CHAIN_S01 DONE
