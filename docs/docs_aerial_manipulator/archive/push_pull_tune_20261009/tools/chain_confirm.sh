#!/bin/bash
# 2026-10-09 PUSH tune, confirmation: the screening's winners repeat-tested (the 2026-08-10 lesson:
# one run proves nothing on this rig), alone and combined, plus a softer bound on K_y.
# Waits for the screening chain to finish.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_tune_20261009
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_TUNE DONE" $C/runs_chain_tune.log 2>/dev/null; do sleep 30; done
run() { tag=$1; shift; clean; bash $C/tools/fly.sh $tag "$@"; }
KY120=(wb_ky_x=120 wb_ky_y=120 wb_ky_z=120 wb_dy_x=20.2 wb_dy_y=20.2 wb_dy_z=20.2)
KY80=(wb_ky_x=80 wb_ky_y=80 wb_ky_z=80 wb_dy_x=16.5 wb_dy_y=16.5 wb_dy_z=16.5)
FF01=(omega_x_t=0.1 omega_x_r=0.1 omega_x_q=0.1)
run c_combo_1  "${KY120[@]}" "${FF01[@]}"
run c_ky120_2  "${KY120[@]}"
run c_combo_2  "${KY120[@]}" "${FF01[@]}"
run c_ky120_3  "${KY120[@]}"
run c_combo_3  "${KY120[@]}" "${FF01[@]}"
run c_ky80_1   "${KY80[@]}"
run c_ky80_2   "${KY80[@]}"
run c_ff01_2   "${FF01[@]}"
clean
echo CHAIN_CONFIRM DONE
