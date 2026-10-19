#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 2: the push distance at the chosen 500 g, the
# speed fixed at the flown 0.5 m / 24 s (the time scales with the distance), the box
# centred on the table (start y = +DIST/2). 0.50 m has 3 runs from stage 1b.
# Waits for chain_s4a AND the flag file PLANNER_INSTALLED (the guards planner installed).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S4A DONE" $C/runs_chain_s4a.log 2>/dev/null || [ ! -f $C/PLANNER_INSTALLED ]; do sleep 30; done
run() { tag=$1; shift; clean; env "$@" bash $C/tools/fly.sh $tag; }
run s2_d070_1 MASS=0.500 DIST=0.70 BOX_Y0=0.36
run s2_d030_1 MASS=0.500 DIST=0.30
run s2_d070_2 MASS=0.500 DIST=0.70 BOX_Y0=0.36
run s2_d030_2 MASS=0.500 DIST=0.30
run s2_d060_1 MASS=0.500 DIST=0.60
run s2_d060_2 MASS=0.500 DIST=0.60
clean
echo CHAIN_S2 DONE
