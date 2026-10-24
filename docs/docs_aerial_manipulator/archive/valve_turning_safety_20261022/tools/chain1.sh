#!/bin/bash
# 2026-10-22 -- the valve turn with the box push's guards (the regenerated valve yaml) and the arm
# GS's automatic Ready / Turn / Exit, headless Isaac, the measured-bearing scene defaults.
#   setsid nohup bash chain1.sh > ../runs/chain1.log 2>&1 &
# Phase 1: nominal, the station flies the steps (as on hardware)        s22_nom_1..3
# Phase 2: faults -- the drone's mocap frozen 3 s mid-turn              s22_mf_drone
#                    the rotor's mocap frozen 2 s mid-turn              s22_mf_rotor
#                    one rotor mis-solve (a 0.18 m / 110 deg jump)     s22_jump_rotor
#                    a heavy valve (damping 2.0 N.m.s/rad, stress)      s22_heavy
# Phase 3: the hardware-like drift model (PEGASUS_PLANT_FORCE_NOISE, the box push's 10-22 fit)
#                                                                        s22_hw_1..3
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PL_RUNS_DIR="$HERE/../runs"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
# the station runs Ready / Turn / Exit (the GS's valve exit: open world-held, Exit 1 s after OPEN)
gs() { tag=$1; shift; [[ -e "$HERE/../runs/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
for k in 1 2 3; do gs s22_nom_$k; done
EXTRA="--mocap-fault freeze@push+12:3" gs s22_mf_drone
EXTRA="--mocap-fault freeze@push+12:2,obj_0" gs s22_mf_rotor
EXTRA="--mocap-fault jump@push+10:0,obj_0" gs s22_jump_rotor
PEGASUS_VALVE_DAMPING=2.0 gs s22_heavy
for k in 1 2 3; do PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs s22_hw_$k; done
clean
echo "CHAIN1 DONE"
