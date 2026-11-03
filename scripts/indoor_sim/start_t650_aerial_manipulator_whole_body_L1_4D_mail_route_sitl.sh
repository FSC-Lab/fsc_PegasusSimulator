#!/usr/bin/env bash
# MAIL-ROUTE SCENE (2026-11-03) for the whole-body L1 4-D rig: three letters on three
# hats, posted alternately into the two mailboxes (letter 1 RED, letter 2 BLUE,
# letter 3 RED) with the planner's mail route (pick_place_route) from the arm GS's
# "Mail Route" tab.
#
# A COPY of start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh
# (its mail_delivery path), which is not touched: it swaps the Isaac app for
# application/robotic_arm/08_px4_t650_aerial_manipulator_mail_route.py and hands over
# to the 4-D launcher. Everything else -- the plant knobs from the paired yaml, the
# 4-D node check, the torque-mode arm stack, both ground stations, the gamepad --
# is that launcher's, unchanged.
#
# The STACK first, on the route's yaml (the mail-delivery profile, its yaml swapped):
#   WB_SIM_PROFILE=mail_delivery \
#   WB_SIM_YAML=$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_route.yaml \
#     start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh
# then this, with the same WB_SIM_YAML:
#   PNP_FEEDBACK=fused (default)  EKF2-fused odometry, the hardware path
#   PNP_FEEDBACK=raw              raw mocap odometry
#
# Usage: start_t650_aerial_manipulator_whole_body_L1_4D_mail_route_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
FOUR_D_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
FUSED_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_fused_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/08_px4_t650_aerial_manipulator_mail_route.py"
export AM_ISAAC_SCENE_LABEL="${AM_ISAAC_SCENE_LABEL:-AM-T650-WB-L1-4D-MAILROUTE}"
# the localization frame is obj_0 in this scene; the EE marker cube would publish it too
export PEGASUS_EE_MARKER_CUBE=0
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-mail_delivery}"
unset PEGASUS_PNP_TASK
# the arm ground station opens on its Mail Route tab (ARM_GS_START_TAB overrides)
export ARM_GS_START_TAB="${ARM_GS_START_TAB:-9}"
[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }

# The scene's knobs must reach the Isaac pane, which tmux starts from its SERVER's
# environment (the stack started the server): push the set ones, clear the unset ones.
if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_PNP_GRIP_FRICTION PEGASUS_PNP_HAT_USD PEGASUS_PNP_GRIP_TORQUE PEGASUS_PNP_WAYPOINTS PEGASUS_PNP_SPAWN_XY PEGASUS_PNP_SPAWN_YAW_DEG PEGASUS_ROUTE_HAT_LINE PEGASUS_MAIL_ENVELOPE_SIZE PEGASUS_MAIL_ENVELOPE_MASS PEGASUS_MAIL_ENVELOPE_COM PEGASUS_MAIL_OVERHANG PEGASUS_MAIL_ENVELOPE_YAW_DEG PEGASUS_MAIL_GRIP_MM PEGASUS_MAIL_PILLAR_HEIGHT PEGASUS_MAIL_TABLE_FRONT_X PEGASUS_MAIL_TABLE_Y PEGASUS_MAIL_RED_Y PEGASUS_MAIL_BLUE_Y PEGASUS_MAIL_BOX_SIZE PEGASUS_MAIL_WALL PEGASUS_MAIL_MOUTH PEGASUS_MAIL_Q2_LIMIT_DEG PEGASUS_MAIL_AM_ASSET PEGASUS_MAIL_FRAME PEGASUS_MAIL_FRAME_AUTO_OFF PEGASUS_MAIL_FRAME_AUTO_MOVE; do
    if [[ -n "${!v:-}" ]]; then
      tmux setenv -g "$v" "${!v}"
      echo -e "\033[1;33m  scene knob $v=${!v}\033[0m"
    else
      tmux setenv -gu "$v" 2>/dev/null || true
    fi
  done
  # (the pick-and-place / mail scenes' own knobs must not leak in from an earlier run)
  for v in PEGASUS_PNP_TASK PEGASUS_MAIL_PILLAR_XY; do tmux setenv -gu "$v" 2>/dev/null || true; done
fi

# The planner must be running the ROUTE's yaml (else it flies the single mission, or
# the route keys' defaults): compare the running planner with it and REFUSE on a
# mismatch. PP_CHECK=0 skips it.
PP_PLANNER_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_route.yaml}"
grep -q '^\s*pick_place_route:\s*true' "$PP_PLANNER_YAML" 2>/dev/null || {
  echo -e "\033[1;31mERROR: $PP_PLANNER_YAML does not switch the mail route on (pick_place_route: true) -- set WB_SIM_YAML to the route yaml\033[0m" >&2
  exit 1; }
if [[ "${PP_CHECK:-1}" != 0 ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/pick_place_planner_check.sh"
  set +e; _pp_out=$(pick_place_planner_check "$PP_PLANNER_YAML" "$PP_PLANNER_YAML"); _pp_rc=$?; set -e
  if [[ $_pp_rc == 0 ]]; then
    echo -e "\033[1;32mplanner: the mail route's block loaded (planner poses and waypoints + controller match $(basename "$PP_PLANNER_YAML"))\033[0m"
  elif [[ $_pp_rc == 1 ]]; then
    echo -e "\033[1;31m$_pp_out\033[0m"
    echo -e "\033[1;31mREFUSED: the running planner / controller are not $(basename "$PP_PLANNER_YAML")'s -- start the stack\033[0m"
    echo -e "\033[1;31mwith WB_SIM_PROFILE=mail_delivery WB_SIM_YAML=$PP_PLANNER_YAML (clean slate first).\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the planner's mail route block ($_pp_out) -- start the stack with WB_SIM_YAML=$PP_PLANNER_YAML first.\033[0m"
  fi
fi

case "${PNP_FEEDBACK:-fused}" in
  fused) LAUNCHER="$FUSED_LAUNCHER" ;;
  raw)   LAUNCHER="$FOUR_D_LAUNCHER" ;;
  *) echo "ERROR: PNP_FEEDBACK must be fused or raw (got '${PNP_FEEDBACK}')" >&2; exit 1 ;;
esac
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

echo -e "\033[1;35mMAIL-ROUTE SCENE ($AM_ISAAC_SCENE_LABEL, ${PNP_FEEDBACK:-fused} feedback): THREE red envelopes, turned ${PEGASUS_MAIL_ENVELOPE_YAW_DEG:-90} deg, on three hats in a line (nearest x, y, spacing ${PEGASUS_ROUTE_HAT_LINE:-0.70,0.0,0.6}) in front of the console table (front x ${PEGASUS_MAIL_TABLE_FRONT_X:-1.70}); letters 1 and 3 into the RED mailbox, letter 2 into the BLUE one; vehicle at ${PEGASUS_PNP_SPAWN_XY:--0.5,-1.0} facing +x. Mocap: /obj_0/mocap = the LOCALIZATION FRAME (on hat 1 at start; it follows the five Gets -- hat 1, 2, 3, RED, BLUE -- or /mail_frame/on_hat_1..3, /on_red, /on_blue, /remove). Arm GS: the Mail Route tab.\033[0m"
exec "$LAUNCHER" "$@"
