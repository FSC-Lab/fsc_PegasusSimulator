#!/usr/bin/env bash
# VALVE-TURNING SCENE (the task of 2026-10-15) on the COORDINATED-ADMITTANCE
# comparison rig (Liu et al., CEP 2024; 2026-10-10; this wrapper 2026-10-17).
#
# The twin of start_t650_aerial_manipulator_whole_body_L1_4D_valve_turning_sitl.sh
# for the other controller: it swaps the Isaac app for the 07 valve scene (the console
# table + the bearing-mounted valve handle, obj_0 = the rotor), pushes the scene knobs into tmux,
# checks that the RUNNING planner and controller came from this rig's valve-turning
# yaml (vehicle_name AM-T650-CADM-VALVE; the planner block is the whole-body valve one
# verbatim) and hands over to the coordinated-admittance launcher (position-mode
# arm, raw mocap feedback -- the only feedback this scene is flown with).
#
#   WB_SIM_PROFILE=valve_turning start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh first
#
# Usage: start_t650_aerial_manipulator_coordinated_admittance_valve_turning_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_coordinated_admittance_direct_actuation_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_t650_aerial_manipulator_valve_turning.py"
export AM_ISAAC_SCENE_LABEL="AM-T650-CADM-VALVE"
export PEGASUS_EE_MARKER_CUBE=0
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-valve_turning}"
export ARM_GS_START_TAB="${ARM_GS_START_TAB:-7}"      # the arm GS on its Valve Turning tab (if the launcher passes it on)

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_VALVE_XY PEGASUS_VALVE_YAW_DEG PEGASUS_VALVE_RISER PEGASUS_VALVE_ROTOR_MASS PEGASUS_VALVE_DAMPING PEGASUS_VALVE_FRICTION PEGASUS_VALVE_FIN_GRIP_MM PEGASUS_VALVE_USD PEGASUS_VALVE_GRIP_TORQUE PEGASUS_VALVE_SPAWN_XY PEGASUS_VALVE_SPAWN_YAW_DEG PEGASUS_VALVE_WAYPOINTS PEGASUS_VALVE_AM_ASSET PEGASUS_VALVE_GROUND_BODY_Z PEGASUS_VALVE_Q2_LIMIT_DEG; do
    if [[ -n "${!v:-}" ]]; then
      tmux setenv -g "$v" "${!v}"
      echo -e "\033[1;33m  scene knob $v=${!v}\033[0m"
    else
      tmux setenv -gu "$v" 2>/dev/null || true
    fi
  done
fi

PL_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650_sim_valve_turning.yaml}"
if [[ "${PL_CHECK:-1}" != 0 && "$WB_SIM_PROFILE" == valve_turning ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/push_pull_planner_check.sh"
  set +e; _pl_out=$(push_pull_planner_check "$PL_YAML"); _pl_rc=$?; set -e
  if [[ $_pl_rc == 0 ]]; then
    echo -e "\033[1;32mplanner + controller: the valve-turning block is loaded (they match $(basename "$PL_YAML"))\033[0m"
  elif [[ $_pl_rc == 1 ]]; then
    echo -e "\033[1;31m$_pl_out\033[0m"
    echo -e "\033[1;31mREFUSED: start the coordinated-admittance stack with WB_SIM_PROFILE=valve_turning first.\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the valve-turning block ($_pl_out).\033[0m"
  fi
fi

echo -e "\033[1;35mVALVE-TURNING SCENE on the COORDINATED-ADMITTANCE rig ($AM_ISAAC_SCENE_LABEL, raw mocap feedback): the same table, valve (bearing friction ${PEGASUS_VALVE_FRICTION:-0.048} N.m, rotor ${PEGASUS_VALVE_ROTOR_MASS:-0.204} kg) and mission (a 360 deg turn) as the whole-body rig.\033[0m"
exec "$LAUNCHER" "$@"
