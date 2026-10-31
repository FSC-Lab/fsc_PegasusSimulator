#!/bin/bash
# The 10-30 item 3 check (2026-10-31): chain.sh's vs_safety was refused DIRECT (arm error 0.24 rad >
# the 0.05 gate) -- with the node's hold stream off, Isaac's arm controller holds the arm with its kp-3
# fallback PD, which sags under gravity. Attempt 2 (vs_safety2) ran arm_hold_kp30.yaml (kp 30 / kd 1.2):
# UNSTABLE in Isaac's arm loop (the arm at the 3 N.m clamp, q4 spinning, the hover never settled).
# Attempt 3 (this): the default arm config, and the DIRECT-entry arm gate widened to 0.35 rad for this
# run only, so the law takes over the sagged arm.
#   setsid nohup bash rerun_safety.sh > rerun_safety.out 2>&1 &
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/runs"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
unset PEGASUS_PLANT_FORCE_NOISE
DRV=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/wb_valve_turning_flight_20261028/tools/driver
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
clean
( source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml
  setsid nohup /usr/bin/python3 "$HERE/force_safety.py" 5.0 "$R/vs_safety3.force.log" > /dev/null 2>&1 & echo $! > "$R/vs_safety3.force.pid" )
echo "$(date +%H:%M:%S) start vs_safety3 wb_arm_hold_stream=false wb_gate_arm_rad=0.35"
ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
  PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 6" \
  bash $C/tools/try_valve.sh vs_safety3 wb_arm_hold_stream=false wb_gate_arm_rad=0.35
P=$(cat "$R/vs_safety3.force.pid" 2>/dev/null); [[ -n "$P" ]] && kill "$P" 2>/dev/null
clean
echo "$(date +%H:%M:%S) RERUN DONE"
