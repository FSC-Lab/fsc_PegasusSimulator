#!/usr/bin/env bash
# BOX-PUSHING SCENE (the task of 2026-10-03/24) on the ARBITRARY-TIME comparison
# rig (Chen et al., IEEE TIE 72(12) 2025; 2026-10-24).
#
# The twin of start_t650_aerial_manipulator_coordinated_admittance_box_pushing_sitl.sh
# for this controller: it swaps the Isaac app for 07 (the console table + the CAD
# box with its fin handle, obj_0), pushes the scene knobs into tmux, checks that
# the RUNNING planner and controller came from this rig's box-pushing yaml
# (vehicle_name AM-T650-ATC-PUSH; the planner block is the whole-body one
# verbatim) and hands over to the arbitrary-time launcher (torque-mode arm,
# raw mocap feedback -- the only feedback this scene is flown with).
#
#   start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh first
#
# Usage: start_t650_aerial_manipulator_arbitrary_time_box_pushing_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_arbitrary_time_direct_actuation_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_t650_aerial_manipulator_box_pushing.py"
export AM_ISAAC_SCENE_LABEL="AM-T650-ATC-PUSH"
export PEGASUS_EE_MARKER_CUBE=0
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-box_pushing}"
export ARM_GS_START_TAB="${ARM_GS_START_TAB:-6}"    # the arm ground station's Box Pushing tab

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_PUSH_BOX_MASS PEGASUS_PUSH_FRICTION_STATIC PEGASUS_PUSH_FRICTION_DYNAMIC PEGASUS_PUSH_BOX_XY PEGASUS_PUSH_BOX_YAW_DEG PEGASUS_PUSH_HANDLE PEGASUS_PUSH_BOX_USD PEGASUS_PUSH_FIN_GRIP_MM PEGASUS_PUSH_POST_TOP PEGASUS_PUSH_AM_ASSET PEGASUS_PUSH_GROUND_BODY_Z PEGASUS_PUSH_Q2_LIMIT_DEG PEGASUS_PUSH_HANDLE_THICKNESS PEGASUS_PUSH_GRIP_TORQUE PEGASUS_PUSH_SPAWN_XY PEGASUS_PUSH_SPAWN_YAW_DEG PEGASUS_PUSH_LAYOUT PEGASUS_PUSH_TABLE_POSE PEGASUS_PUSH_WAYPOINTS PEGASUS_PUSH_STOPPER_Y; do
    if [[ -n "${!v:-}" ]]; then
      tmux setenv -g "$v" "${!v}"
      echo -e "\033[1;33m  scene knob $v=${!v}\033[0m"
    else
      tmux setenv -gu "$v" 2>/dev/null || true
    fi
  done
fi

PL_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_arbitrary_time_direct_actuation_t650_sim_box_pushing.yaml}"
if [[ "${PL_CHECK:-1}" != 0 ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/push_pull_planner_check.sh"
  set +e; _pl_out=$(push_pull_planner_check "$PL_YAML"); _pl_rc=$?; set -e
  if [[ $_pl_rc == 0 ]]; then
    echo -e "\033[1;32mplanner + controller: the box-pushing block is loaded (they match $(basename "$PL_YAML"))\033[0m"
  elif [[ $_pl_rc == 1 ]]; then
    echo -e "\033[1;31m$_pl_out\033[0m"
    echo -e "\033[1;31mREFUSED: start the arbitrary-time stack (start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh) first.\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the box-pushing block ($_pl_out).\033[0m"
  fi
fi

echo -e "\033[1;35mBOX-PUSHING SCENE on the ARBITRARY-TIME rig ($AM_ISAAC_SCENE_LABEL, raw mocap feedback): the same table, box (${PEGASUS_PUSH_BOX_MASS:-scene default} kg, ${PEGASUS_PUSH_HANDLE:-cad} handle) and mission as the whole-body rig.\033[0m"
exec "$LAUNCHER" "$@"
