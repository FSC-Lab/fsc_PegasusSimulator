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
# 2026-10-28 night: (a) the REDESIGNED valve markers (Valve_Assembly_v2.usda regenerated, axis offset
# 2026-10-28 night, round 4 (all with the exit hold, the 4 deg tilt limit and the redesigned markers; the
# valve driver copy, whose hover check low-passes the velocity):
#  (1) the drone's raw mocap at its hardware CONTACT level (pos 1.5 mm, vel 60 mm/s per axis) -- does the
#      hardware's growing 0.56-0.6 Hz lateral sway appear?
#  (2) wb_l1_omega_c_t 2.0 -> 2.927 (the free-flight tune): less drift for the arm to absorb (10-18: lead
#      -20 %), rejected then only for the exit the exit hold now fixes
#  (3) wb_ee_anchor_free_axis_weight 0.75 -> 1.0: the arm absorbs none of the fore-aft drift
for k in 2 3; do export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k"
  gs nz_s$k sim_feedback_pos_noise_m=0.0015 sim_feedback_vel_noise_mps=0.06; done
for k in 2 3 4; do export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k"
  gs oc29_s$k wb_l1_omega_c_t=2.927
  gs fa10_s$k wb_ee_anchor_free_axis_weight=1.0; done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
