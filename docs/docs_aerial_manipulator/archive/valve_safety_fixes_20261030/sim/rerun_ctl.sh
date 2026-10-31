#!/bin/bash
# (2026-10-31) The CONTROL for vs_safety3: the same forced SAFETY 5 s into the turn, but the shipped
# sim setting wb_arm_hold_stream true (the node streams its PD+gravity arm hold in SAFETY -- no RELEASE).
# Separates what the release does from what the SAFETY handover in contact does.
#   setsid nohup bash rerun_ctl.sh > rerun_ctl.out 2>&1 &
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
  setsid nohup /usr/bin/python3 "$HERE/force_safety.py" 5.0 "$R/vs_safety_ctl.force.log" > /dev/null 2>&1 & echo $! > "$R/vs_safety_ctl.force.pid" )
echo "$(date +%H:%M:%S) start vs_safety_ctl (the node streams the arm hold: no RELEASE)"
ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
  PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 6" \
  bash $C/tools/try_valve.sh vs_safety_ctl
P=$(cat "$R/vs_safety_ctl.force.pid" 2>/dev/null); [[ -n "$P" ]] && kill "$P" 2>/dev/null
clean
echo "$(date +%H:%M:%S) RERUN DONE"
