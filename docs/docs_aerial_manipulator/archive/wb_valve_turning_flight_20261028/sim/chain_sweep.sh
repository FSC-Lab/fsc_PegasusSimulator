#!/bin/bash
# (harness as chain.sh: v2 valve at 0.072 N.m drive, GS-run steps, each run stops 4 s after the exit)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/runs"; mkdir -p "$R"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; echo "$(date +%H:%M:%S) start $tag $*"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$HERE/../tools/driver" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" bash $C/tools/try_valve.sh "$tag" "$@"; }
# 2026-10-29, quick sweep (user): impedance (K_y/D_y soft + stiff, damping ratio kept), heading
# impedance (K_psi 1.0), the turn reference (90 deg in 15 s). Baseline = last night's qg_s2 / qg_s3
# (the same config the generator writes today). Drift seeds 2 and 3.
for k in 2 3; do export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k"
  gs sw_ky80_s$k  wb_ky_x=80.0 wb_ky_y=80.0 wb_ky_z=80.0 wb_dy_x=16.5 wb_dy_y=16.5 wb_dy_z=16.5
  gs sw_ky160_s$k wb_ky_x=160.0 wb_ky_y=160.0 wb_ky_z=160.0 wb_dy_x=23.3 wb_dy_y=23.3 wb_dy_z=23.3
  gs sw_psi1_s$k  wb_ky_psi=1.0 wb_dy_psi=0.582
  gs sw_t15_s$k   push_pull_push_time_s=15.0
done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
