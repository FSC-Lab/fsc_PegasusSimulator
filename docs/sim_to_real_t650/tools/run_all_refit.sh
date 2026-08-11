#!/usr/bin/env bash
# Validate the 2026-08-07 refit on all three flights with ONE parameter set:
# flight C in baseline mode, D1 and D2 in DIRECT mode.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
bash ./run_stack_case.sh  ../data/ref_C.npz  ../data/sim_stack_C_refit.npz   Crefit > logs_C_console.log  2>&1
sleep 15
bash ./run_direct_case.sh ../data/ref_D1.npz ../data/sim_direct_D1_refit.npz D1r    > logs_D1r_console.log 2>&1
sleep 15
bash ./run_direct_case.sh ../data/ref_D2.npz ../data/sim_direct_D2_refit.npz D2r    > logs_D2r_console.log 2>&1
echo "ALL THREE DONE"
