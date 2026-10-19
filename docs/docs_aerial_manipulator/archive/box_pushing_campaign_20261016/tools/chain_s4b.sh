#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 4b: the worst cases again, at the FINAL configuration
# (500 g, 0.50 m, the stage-3 settings baked into make_push_pull_yaml.py) with the new box
# guards ON, plus the mass / friction robustness of the final config. Waits for the flag file
# FINAL_CONFIG (written once the generator carries the stage-3 result).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while [ ! -f $C/FINAL_CONFIG ]; do sleep 30; done
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
NOG=(push_pull_guard_box_lag=0.0 push_pull_guard_box_tilt_deg=0.0 push_pull_guard_q3_min_deg=0.0)
R g_stop      "PEGASUS_PUSH_STOPPER_Y=-0.07"
R g_heavy     "MASS=1.000 MU_S=0.6 MU_K=0.6"
R g_stick     "MU_S=0.5 MU_K=0.29"
R g_stick_nog "MU_S=0.5 MU_K=0.29" "${NOG[@]}"
R g_flip      "PEGASUS_PUSH_BOX_YAW_DEG=180"
R g_far       "DIST=0.90 BOX_Y0=0.25"
R g_abort     "PL_DRIVER_ARGS=--inject=abort@10"
R g_m400      "MASS=0.400"
R g_m600      "MASS=0.600"
R g_mu035     "MU_S=0.35 MU_K=0.35"
clean
echo CHAIN_S4B DONE
