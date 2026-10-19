#!/bin/bash
# 2026-10-17: the min-snap push flown TODAY on today's build -- the control for the cruise runs
# (2 of 3 cruise pushes ended in a q3-guard abort after the release, one of them a crash).
# Stop with the process group (kill -- -PGID), never this bash alone.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_cruise_20261017
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R m_nom_1 "T=24 RAMP=0"
R m_nom_2 "T=24 RAMP=0"
clean
echo CHAIN_CONTROL DONE
