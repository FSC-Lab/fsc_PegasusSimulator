#!/bin/bash
# 2026-10-17 follow-on: c_nom_2 was aborted by the q3 guard AFTER the release (the base drifted
# 46 mm while the open claw stayed world-held). Two more cruise pushes, and two min-snap pushes
# flown TODAY on the same build as a same-day control. Waits for chain_cruise_rest.sh.
# Stop with the process group (kill -- -PGID), never this bash alone.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_cruise_20261017
until grep -q "CHAIN_CRUISE DONE" $C/runs_chain.log 2>/dev/null; do sleep 20; done
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R c_nom_4 "T=20 RAMP=3"
R m_nom_1 "T=24 RAMP=0"
R c_nom_5 "T=20 RAMP=3"
R m_nom_2 "T=24 RAMP=0"
clean
echo CHAIN_REPEAT DONE
