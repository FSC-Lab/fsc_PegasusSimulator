#!/usr/bin/env bash
# VALVE-TURNING SCENE (2026-10-15) for the whole-body L1 4-D rig.
#
# A thin wrapper, the box-pushing wrapper's twin: it swaps the Isaac app for
# application/robotic_arm/07_px4_t650_aerial_manipulator_valve_turning.py
# (06's free-flight plant + the HOOBRO console table with the user's valve
# mechanism bolted to it -- a beam on a bearing with a fin handle, the rotor
# published as the mocap body obj_0) and hands over to the 4-D launcher.
# Everything else -- the plant knobs from the paired yaml, the 4-D node check,
# the torque-mode arm stack, both ground stations, the gamepad -- is that
# launcher's, unchanged. The mission is the push-and-pull mode's with
# push_pull_turn_deg 360: Push TURNS the handle a full revolution.
#
#   VALVE_FEEDBACK=raw (default)   raw mocap odometry (the box push's choice: in
#       contact EKF2 drifts 1-1.5 cm and every fused push failed, 2026-10-05):
#       WB_SIM_PROFILE=valve_turning start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh first
#   VALVE_FEEDBACK=fused           EKF2-fused odometry, the hardware path:
#       WB_SIM_PROFILE=valve_turning start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh first
#
# Usage: start_t650_aerial_manipulator_whole_body_L1_4D_valve_turning_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
FOUR_D_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
FUSED_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_fused_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_t650_aerial_manipulator_valve_turning.py"
export AM_ISAAC_SCENE_LABEL="AM-T650-WB-L1-4D-VALVE"
# The rotor is obj_0 in this scene; the EE marker cube would publish it too.
export PEGASUS_EE_MARKER_CUBE=0
# The arm ground station opens on its Valve Turning tab (index 7, 2026-10-16;
# ARM_GS_START_TAB overrides).
export ARM_GS_START_TAB="${ARM_GS_START_TAB:-7}"
# The paired config: the mirror plant + the box push's controller tune with the
# push-and-pull planner block in its TURN form (..._sim_valve_turning.yaml). The
# STACK must be started with the same profile -- this launcher checks below.
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-valve_turning}"

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }

# The scene's knobs (07's PEGASUS_VALVE_*) must reach the Isaac pane, which tmux
# starts from its SERVER's environment -- and the server is already up (the
# stack started it). Push the set ones into the global environment and clear
# the unset ones, so a knob from an earlier run cannot leak into this one.
if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_VALVE_XY PEGASUS_VALVE_YAW_DEG PEGASUS_VALVE_RISER PEGASUS_VALVE_ROTOR_MASS PEGASUS_VALVE_DAMPING PEGASUS_VALVE_FRICTION PEGASUS_VALVE_FIN_GRIP_MM PEGASUS_VALVE_USD PEGASUS_VALVE_GRIP_TORQUE PEGASUS_VALVE_SPAWN_XY PEGASUS_VALVE_SPAWN_YAW_DEG PEGASUS_VALVE_WAYPOINTS PEGASUS_VALVE_AM_ASSET PEGASUS_VALVE_GROUND_BODY_Z PEGASUS_VALVE_Q2_LIMIT_DEG PEGASUS_VALVE_LOAD_TORQUE PEGASUS_VALVE_LOAD_DIR PEGASUS_VALVE_LOAD_TAKEUP_DEG PEGASUS_VALVE_RATCHET; do
    if [[ -n "${!v:-}" ]]; then
      tmux setenv -g "$v" "${!v}"
      echo -e "\033[1;33m  scene knob $v=${!v}\033[0m"
    else
      tmux setenv -gu "$v" 2>/dev/null || true
    fi
  done
fi

# The planner reads its push-and-pull block once, at start: a stack started
# without WB_SIM_PROFILE=valve_turning flies the node defaults (EE offset 0 --
# the claw INTO the bearing --, no turn, no table check) or the BOX PUSH's
# block (a straight 0.5 m push through the beam). Compare the running planner
# and controller with the valve yaml and REFUSE on a mismatch (the shared
# check compares push_pull_turn_deg and push_pull_obstacle too). PL_CHECK=0 skips it.
PL_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_valve_turning.yaml}"
if [[ "${PL_CHECK:-1}" != 0 && "$WB_SIM_PROFILE" == valve_turning ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/push_pull_planner_check.sh"
  set +e; _pl_out=$(push_pull_planner_check "$PL_YAML"); _pl_rc=$?; set -e
  if [[ $_pl_rc == 0 ]]; then
    echo -e "\033[1;32mplanner + controller: the valve-turning block is loaded (they match the valve-turning yaml)\033[0m"
  elif [[ $_pl_rc == 1 ]]; then
    echo -e "\033[1;31m$_pl_out\033[0m"
    echo -e "\033[1;31mREFUSED: the running stack was not started with WB_SIM_PROFILE=valve_turning (or the planner\033[0m"
    echo -e "\033[1;31mbinary predates push_pull_turn_deg). Clean slate, then start the stack with it (Command.md section 18).\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the valve-turning block ($_pl_out) -- start the stack with WB_SIM_PROFILE=valve_turning first.\033[0m"
  fi
fi

case "${VALVE_FEEDBACK:-raw}" in
  fused) LAUNCHER="$FUSED_LAUNCHER" ;;
  raw)   LAUNCHER="$FOUR_D_LAUNCHER" ;;
  *) echo "ERROR: VALVE_FEEDBACK must be raw or fused (got '${VALVE_FEEDBACK}')" >&2; exit 1 ;;
esac
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

echo -e "\033[1;35mVALVE-TURNING SCENE ($AM_ISAAC_SCENE_LABEL, ${VALVE_FEEDBACK:-raw} feedback): vehicle at ${PEGASUS_VALVE_SPAWN_XY:-0.10,-0.07} facing +x; the HOOBRO console table (1.000 x 0.150 x 0.800 m) centred on (1.20, 0), its length along y; the valve's bearing axis at ${PEGASUS_VALVE_XY:-1.20,0.00} on the table top, the beam pointing ${PEGASUS_VALVE_YAW_DEG:-180} deg, the fin at radius 0.160 m (grip shim ${PEGASUS_VALVE_FIN_GRIP_MM:-20} mm), bearing damping ${PEGASUS_VALVE_DAMPING:-0.0} N.m.s/rad + friction ${PEGASUS_VALVE_FRICTION:-0.048} N.m, rotor ${PEGASUS_VALVE_ROTOR_MASS:-0.204} kg. Mocap: /obj_0/mocap = the rotor (axis + the beam's yaw).\033[0m"
exec "$LAUNCHER" "$@"
