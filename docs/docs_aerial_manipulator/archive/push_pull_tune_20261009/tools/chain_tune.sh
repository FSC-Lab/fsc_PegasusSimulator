#!/bin/bash
# 2026-10-09 PUSH tune, screening: one change per run against the shipped push yaml (the A/B's
# ab_push_1..3 are the baseline), the groups that act on a contact task first. Waits for the
# A/B chain to finish (its log's CHAIN_AB DONE line). PULL=1 flies the same sweep as pulls.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_tune_20261009
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_AB DONE" $C/runs_chain_ab.log 2>/dev/null; do sleep 30; done
run() { tag=$1; shift; clean; bash $C/tools/fly.sh $tag "$@"; }
# A. the EE impedance itself (K_y / D_y at the same damping ratio 0.92)
run t_ky120   wb_ky_x=120 wb_ky_y=120 wb_ky_z=120 wb_dy_x=20.2 wb_dy_y=20.2 wb_dy_z=20.2
run t_ky300   wb_ky_x=300 wb_ky_y=300 wb_ky_z=300 wb_dy_x=31.9 wb_dy_y=31.9 wb_dy_z=31.9
# B. the translational observer (1.0 shipped; 2.927 = the mirror, fails in range)
run t_oct05   omega_c_t=0.5
run t_oct15   omega_c_t=1.5
# C. the rotational observer (0.7428 shipped)
run t_ocr05   omega_c_r=0.5
run t_ocr10   omega_c_r=1.0
# D. the contact-force reading filter (2.0 shipped)
run t_ox10    omega_x=1.0
run t_ox40    omega_x=4.0
# E. the u3 feed-forward filters (0.2072 shipped)
run t_oxt01   omega_x_t=0.1 omega_x_r=0.1 omega_x_q=0.1
run t_oxt04   omega_x_t=0.4 omega_x_r=0.4 omega_x_q=0.4
# F. attitude (1.6 / 1.2 shipped)
run t_kr13    wb_k_r=1.3 wb_k_w=1.0
run t_kr21    wb_k_r=2.134 wb_k_w=1.567
# G. position (50.03 / 12.58 shipped)
run t_kx32    wb_k_x=32 wb_k_v=10
run t_kx64    wb_k_x=64 wb_k_v=14
# H. the push speed (24 s shipped)
run t_T18     push_pull_push_time_s=18.0
run t_T12     push_pull_push_time_s=12.0
clean
echo CHAIN_TUNE DONE
