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
    PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" bash $C/tools/try_valve.sh "$tag" "$@"; }
# 2026-10-28 night: (a) the REDESIGNED valve markers (Valve_Assembly_v2.usda regenerated, axis offset
# [-0.0935, -0.0014, -0.0899]) -- nominal + drift seed 2; (b) is the hardware contact sway driven by the
# raw mocap noise? the drone's raw mocap at its measured CONTACT level (hardware 10-28: position >5 Hz
# 1.2-5.3 mm, velocity 57-101 mm/s rms vs 11-17 free) on drift seeds 2 and 3.
unset PEGASUS_PLANT_FORCE_NOISE; gs nm_nom
export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2"; gs nm_s2
gs nz_s2 sim_feedback_pos_noise_m=0.0015 sim_feedback_vel_noise_mps=0.06
export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,3"; gs nz_s3 sim_feedback_pos_noise_m=0.0015 sim_feedback_vel_noise_mps=0.06
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
