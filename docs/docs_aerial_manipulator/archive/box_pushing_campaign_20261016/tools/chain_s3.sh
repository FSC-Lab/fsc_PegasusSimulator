#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 3: the controller / task settings at the chosen
# 500 g and distance, one change at a time, 2 runs each (the 2026-08-10 lesson: one run proves
# nothing on this rig), the new box guards ON in every run (each run is also a false-trip test).
#   P   the push pose re-split [0, 26, 34, 0] -> [0, 32, 28, 0] (same 60 deg fold): q3's +50 stop
#       is the binding margin (the loaded swing rides it at 400-600 g); +6 deg on q2 re-centres
#       q3 ~6 deg further from it while q2 keeps ~4 deg to its +45
#   H   the 2026-10-16 j4 heading fix (wb_ky/dy_psi 2.5 / 0.93 + wb_l1_stuck_hold_j4): the box's
#       yaw drift (5-9 deg) follows the EE heading, the soft axis since 10-04
#   K   K_y / D_y 120 / 20.2 -> 150 / 22.6 (same zeta 0.92): 12 mm of claw deflection at 500 g
# Waits for chain_s2 and the distance (written into DIST_CHOSEN).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S2 DONE" $C/runs_chain_s2.log 2>/dev/null || [ ! -f $C/DIST_CHOSEN ]; do sleep 30; done
D=$(cat $C/DIST_CHOSEN | awk '{print $1}'); Y0=$(cat $C/DIST_CHOSEN | awk '{print $2}')
R() { tag=$1; shift; clean; env MASS=0.500 DIST=$D ${Y0:+BOX_Y0=$Y0} bash $C/tools/fly.sh $tag "$@"; }
P=("push_pull_push_pose_deg=[0.0, 32.0, 28.0, 0.0]")
H=(wb_ky_psi=2.5 wb_dy_psi=0.93 stuck_hold_j4=true)
K=(wb_ky_x=150.0 wb_ky_y=150.0 wb_ky_z=150.0 wb_dy_x=22.6 wb_dy_y=22.6 wb_dy_z=22.6)
R s3_P_1 "${P[@]}"
R s3_H_1 "${H[@]}"
R s3_K_1 "${K[@]}"
R s3_P_2 "${P[@]}"
R s3_H_2 "${H[@]}"
R s3_K_2 "${K[@]}"
clean
echo CHAIN_S3 DONE
