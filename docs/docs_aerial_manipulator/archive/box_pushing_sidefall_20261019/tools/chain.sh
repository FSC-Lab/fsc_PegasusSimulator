#!/bin/bash
# 2026-10-19: a light box (240 g, the real box emptied) and the side-edge fall while clamped.
# Hardware guard values throughout (planner tilt guard 10 deg, abort release hold 1.5 s).
# SIDE FALL (sf*): the box placed at 15 deg yaw with the plan-time on-table check off
# (push_pull_box_half_length 0), so the planned push line runs over the table's SIDE edge
# (+x, CoM over the edge after ~0.29 m) with the jaws closed on the fin.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_sidefall_20261019
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
HW="push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
SF="push_pull_box_half_length=0.0"
R() { tag=$1; envs=$2; shift 2; clean; echo "[$(date +%T)] start $tag"; env DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R sf240_1 "MASS=0.240 PEGASUS_PUSH_BOX_YAW_DEG=15" $HW $SF
R sf500_1 "MASS=0.500 PEGASUS_PUSH_BOX_YAW_DEG=15" $HW $SF
R sf240_2 "MASS=0.240 PEGASUS_PUSH_BOX_YAW_DEG=15" $HW $SF
R sf500_2 "MASS=0.500 PEGASUS_PUSH_BOX_YAW_DEG=15" $HW $SF
R l240_1  "MASS=0.240" $HW
R l240_2  "MASS=0.240" $HW
R l240_3  "MASS=0.240" $HW
clean
echo "[$(date +%T)] CHAIN DONE"
