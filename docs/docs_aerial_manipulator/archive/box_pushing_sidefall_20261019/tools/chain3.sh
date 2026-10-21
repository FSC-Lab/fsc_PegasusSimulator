#!/bin/bash
# 2026-10-19, after chain2.sh: re-fly the nominal 240 g push l240_2 (its arm stack never
# came up -- DIRECT entry refused before take-off, not a flight result).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_sidefall_20261019
while ! grep -q "CHAIN2 DONE" $C/runs_chain2.log 2>/dev/null; do sleep 20; done
sleep 10
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
R() { tag=$1; envs=$2; shift 2; clean; echo "[$(date +%T)] start $tag"; env DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R l240_2b "MASS=0.240" $HW
clean
echo "[$(date +%T)] CHAIN3 DONE"
