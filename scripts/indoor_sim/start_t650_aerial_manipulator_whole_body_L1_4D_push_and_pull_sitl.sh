#!/usr/bin/env bash
# PUSH-AND-PULL SCENE (2026-10-03) for the whole-body L1 4-D rig.
#
# A thin wrapper: it swaps the Isaac app for
# application/robotic_arm/07_px4_direct_t650_aerial_manipulator_push_and_pull.py
# (06's free-flight plant + the HOOBRO console table and the 200 g box with a
# fin handle on its side, published as the mocap body obj_0) and hands over to
# the 4-D launcher. Everything else -- the plant knobs from the paired yaml, the
# 4-D node check, the torque-mode arm stack, both ground stations, the gamepad
# -- is that launcher's, unchanged.
#
#   PUSH_FEEDBACK=fused (default)  EKF2-fused odometry, the hardware path:
#       WB_SIM_PROFILE=push_pull start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh first
#   PUSH_FEEDBACK=raw              raw mocap odometry:
#       WB_SIM_PROFILE=push_pull start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh first
#
# Usage: start_t650_aerial_manipulator_whole_body_L1_4D_push_and_pull_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
FOUR_D_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
FUSED_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_fused_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_direct_t650_aerial_manipulator_push_and_pull.py"
export AM_ISAAC_SCENE_LABEL="AM-T650-WB-L1-4D-PUSH"
# The box is obj_0 in this scene; the EE marker cube would publish it too.
export PEGASUS_EE_MARKER_CUBE=0
# The paired config: the mirror plant + controller with the push-and-pull
# planner block and the contact reading split (..._sim_push_pull.yaml). The
# STACK must be started with the same profile -- this launcher checks below.
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-push_pull}"

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }

# The scene's knobs (07's PEGASUS_PUSH_*) must reach the Isaac pane, which tmux
# starts from its SERVER's environment -- and the server is already up (the
# stack started it). Push the set ones into the global environment and clear
# the unset ones, so a knob from an earlier run cannot leak into this one.
if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_PUSH_BOX_MASS PEGASUS_PUSH_FRICTION_STATIC PEGASUS_PUSH_FRICTION_DYNAMIC PEGASUS_PUSH_BOX_XY PEGASUS_PUSH_BOX_YAW_DEG PEGASUS_PUSH_HANDLE_THICKNESS PEGASUS_PUSH_GRIP_TORQUE PEGASUS_PUSH_SPAWN_XY PEGASUS_PUSH_SPAWN_YAW_DEG PEGASUS_PUSH_WAYPOINTS; do
    if [[ -n "${!v:-}" ]]; then
      tmux setenv -g "$v" "${!v}"
      echo -e "\033[1;33m  scene knob $v=${!v}\033[0m"
    else
      tmux setenv -gu "$v" 2>/dev/null || true
    fi
  done
fi

# The planner reads its push-and-pull block once, at start: a stack started
# without WB_SIM_PROFILE=push_pull flies the node defaults (EE offset 0 -- the
# claw INTO the box --, no table check). Compare the running planner and
# controller with the push-and-pull yaml and REFUSE on a mismatch. PL_CHECK=0
# skips it.
PL_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_push_pull.yaml}"
if [[ "${PL_CHECK:-1}" != 0 && "$WB_SIM_PROFILE" == push_pull ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/push_pull_planner_check.sh"
  set +e; _pl_out=$(push_pull_planner_check "$PL_YAML"); _pl_rc=$?; set -e
  if [[ $_pl_rc == 0 ]]; then
    echo -e "\033[1;32mplanner + controller: the push-and-pull block is loaded (they match the push-and-pull yaml)\033[0m"
  elif [[ $_pl_rc == 1 ]]; then
    echo -e "\033[1;31m$_pl_out\033[0m"
    echo -e "\033[1;31mREFUSED: the running stack was not started with WB_SIM_PROFILE=push_pull (or the planner\033[0m"
    echo -e "\033[1;31mbinary predates push-and-pull). Clean slate, then start the stack with it (Command.md section 17).\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the push-and-pull block ($_pl_out) -- start the stack with WB_SIM_PROFILE=push_pull first.\033[0m"
  fi
fi

case "${PUSH_FEEDBACK:-fused}" in
  fused) LAUNCHER="$FUSED_LAUNCHER" ;;
  raw)   LAUNCHER="$FOUR_D_LAUNCHER" ;;
  *) echo "ERROR: PUSH_FEEDBACK must be fused or raw (got '${PUSH_FEEDBACK}')" >&2; exit 1 ;;
esac
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

echo -e "\033[1;35mPUSH-AND-PULL SCENE ($AM_ISAAC_SCENE_LABEL, ${PUSH_FEEDBACK:-fused} feedback): vehicle at ${PEGASUS_PUSH_SPAWN_XY:-0.10,-0.07} facing +x; the HOOBRO console table (1.000 x 0.150 x 0.800 m) centred on (1.20, 0), its length along y; a ${PEGASUS_PUSH_BOX_MASS:-0.200} kg box (200 x 100 x 60 mm + a 20 mm fin handle on its +y face) at ${PEGASUS_PUSH_BOX_XY:-1.20,0.25}. Mocap: /obj_0/mocap = the box (centre + yaw).\033[0m"
exec "$LAUNCHER" "$@"
