#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 3b: the pose re-split P a third time (confirmation,
# 3 runs for anything shipped), and P + the j4 heading fix H twice (H alone was neutral-to-
# worse in its one completed run; its repeat never flew -- the arm stack did not come up).
# Waits for chain_s3.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S3 DONE" $C/runs_chain_s3.log 2>/dev/null; do sleep 30; done
R() { tag=$1; shift; clean; env MASS=0.500 DIST=0.50 bash $C/tools/fly.sh $tag "$@"; }
P=("push_pull_push_pose_deg=[0.0, 32.0, 28.0, 0.0]")
H=(wb_ky_psi=2.5 wb_dy_psi=0.93 stuck_hold_j4=true)
R s3_P_3  "${P[@]}"
R s3_PH_1 "${P[@]}" "${H[@]}"
R s3_PH_2 "${P[@]}" "${H[@]}"
clean
echo CHAIN_S3B DONE
