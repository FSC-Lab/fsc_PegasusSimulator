#!/bin/bash
# 2026-10-19, after chain.sh: the side-edge fall with the BOX GUARDS OFF (box lag / box tilt
# 0 -> the box-feed staleness check is off with them). What is left is what would act if the
# box guards missed: the planner's vehicle-tilt guard (10 deg), the claw-force guard (5 N),
# the q3 guard (10 deg), and the flight node's watchdog. The box hangs from the claw.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_sidefall_20261019
while ! grep -q "CHAIN DONE" $C/runs_chain.log 2>/dev/null; do sleep 20; done
sleep 10
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
SF="push_pull_box_half_length=0.0"
NG="push_pull_guard_box_lag=0.0 push_pull_guard_box_tilt_deg=0.0"
R() { tag=$1; envs=$2; shift 2; clean; echo "[$(date +%T)] start $tag"; env DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R sf240_ng1 "MASS=0.240 PEGASUS_PUSH_BOX_YAW_DEG=15" $HW $SF $NG
R sf500_ng1 "MASS=0.500 PEGASUS_PUSH_BOX_YAW_DEG=15" $HW $SF $NG
clean
echo "[$(date +%T)] CHAIN2 DONE"
