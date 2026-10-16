#!/usr/bin/env bash
# PICK-AND-PLACE SCENE (2026-10-01) for the whole-body L1 4-D rig.
#
# A thin wrapper: it swaps the Isaac app for
# application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py
# (06's free-flight plant + the field, the two pillars and the 200 g
# payload, published as the mocap bodies obj_0 and drop_0) and hands over to
# the 4-D launcher. Everything else -- the plant knobs from the paired yaml,
# the 4-D node check, the torque-mode arm stack, both ground stations, the
# gamepad -- is that launcher's, unchanged.
#
#   PNP_FEEDBACK=fused (default)  EKF2-fused odometry, the hardware path:
#       WB_SIM_PROFILE=<profile> start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh first
#   PNP_FEEDBACK=raw              raw mocap odometry:
#       WB_SIM_PROFILE=<profile> start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh first
#
# TWO DEMONSTRATIONS (2026-10-14), one WB_SIM_PROFILE each, the SAME on the stack
# and on this wrapper (it sets the scene's PEGASUS_PNP_TASK from it):
#   pick_and_place (default)  the ORIGINAL pick-and-place, the paper's comparison:
#                             one-piece basket, 1.0 m pillars, set down on the hat
#   pick_and_drop             the PICK-AND-DROP: the real payload (Box_Assembly),
#                             0.77 m pillars, dropped into the bin on the place hat
#
# Usage: start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
FOUR_D_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
FUSED_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_fused_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py"
export AM_ISAAC_SCENE_LABEL="${AM_ISAAC_SCENE_LABEL:-AM-T650-WB-L1-4D-PNP}"
# The payload is obj_0 in this scene; the EE marker cube would publish it too.
export PEGASUS_EE_MARKER_CUBE=0
# The paired config: the mirror plant + controller with the tuned pick-and-place
# planner block (..._sim_pick_and_place.yaml). The STACK must be started with the
# same profile -- this launcher checks the running planner below.
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-pick_and_place}"
case "$WB_SIM_PROFILE" in
  pick_and_place|pick_place) _PNP_TASK=place; _PNP_YAML_SUFFIX=_sim_pick_and_place.yaml; _PNP_TAB=4 ;;
  pick_and_drop)             _PNP_TASK=drop;  _PNP_YAML_SUFFIX=_sim_pick_and_drop.yaml;  _PNP_TAB=5 ;;
  *) echo "ERROR: WB_SIM_PROFILE must be pick_and_place or pick_and_drop for this scene (got '$WB_SIM_PROFILE')" >&2; exit 1 ;;
esac
# the scene's demonstration (07: payload, pillar heights, the bin) follows the
# profile unless set explicitly
export PEGASUS_PNP_TASK="${PEGASUS_PNP_TASK:-$_PNP_TASK}"
# ... and the arm ground station opens on that demonstration's tab (Pick & Place /
# Pick & Drop; ARM_GS_START_TAB overrides)
export ARM_GS_START_TAB="${ARM_GS_START_TAB:-$_PNP_TAB}"

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }

# The scene's knobs (07's PEGASUS_PNP_*) must reach the Isaac pane, which tmux
# starts from its SERVER's environment -- and the server is already up (the
# stack started it). Push the set ones into the global environment and clear
# the unset ones, so a knob from an earlier run cannot leak into this one.
# (list-sessions, not `tmux info`: info needs an attached client, and a
# background launch has none -- the knobs then silently never arrive)
if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_PNP_TASK PEGASUS_PNP_BIN PEGASUS_PNP_BIN_SIZE PEGASUS_PNP_BIN_YAW_DEG PEGASUS_PNP_PAYLOAD PEGASUS_PNP_PAYLOAD_USD PEGASUS_PNP_FIN_GRIP_MM PEGASUS_PNP_PICK_PILLAR_HEIGHT PEGASUS_PNP_PLACE_PILLAR_HEIGHT PEGASUS_PNP_GRIP_FRICTION PEGASUS_PNP_PAYLOAD_MASS PEGASUS_PNP_PAYLOAD_YAW_DEG PEGASUS_PNP_HANDLE_THICKNESS PEGASUS_PNP_PLATFORM PEGASUS_PNP_HAT_USD PEGASUS_PNP_CAP_DIAMETER PEGASUS_PNP_GRIP_TORQUE PEGASUS_PNP_GRASP_TEST PEGASUS_PNP_WAYPOINTS PEGASUS_PNP_SPAWN_XY PEGASUS_PNP_SPAWN_YAW_DEG; do
    if [[ -n "${!v:-}" ]]; then
      tmux setenv -g "$v" "${!v}"
      echo -e "\033[1;33m  scene knob $v=${!v}\033[0m"
    else
      tmux setenv -gu "$v" 2>/dev/null || true
    fi
  done
fi

# The planner reads its pick-and-place block once, at start: a stack started
# without WB_SIM_PROFILE=pick_place flies the node defaults (pick / place pose
# [0, 0, 0, 0], carry = the folded home pose, every yaw 0 -- no clockwise
# turns). Compare the running planner with the pick-and-place yaml and REFUSE
# on a mismatch (2026-10-03: the old check on approach_dz <= 0 stopped seeing
# anything once that default became 0.20 m, and a stack without the profile
# was reported as loaded). PP_CHECK=0 skips it.
PP_PLANNER_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650${_PNP_YAML_SUFFIX}}"
if [[ "${PP_CHECK:-1}" != 0 ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/pick_place_planner_check.sh"
  set +e; _pp_out=$(pick_place_planner_check "$PP_PLANNER_YAML" "$PP_PLANNER_YAML"); _pp_rc=$?; set -e
  if [[ $_pp_rc == 0 ]]; then
    echo -e "\033[1;32mplanner: $WB_SIM_PROFILE block loaded (planner poses and waypoints + controller match $(basename "$PP_PLANNER_YAML"))\033[0m"
  elif [[ $_pp_rc == 1 ]]; then
    echo -e "\033[1;31m$_pp_out\033[0m"
    echo -e "\033[1;31mREFUSED: the running planner / controller are not $(basename "$PP_PLANNER_YAML")'s -- the stack was started\033[0m"
    echo -e "\033[1;31mwithout WB_SIM_PROFILE=$WB_SIM_PROFILE. Clean slate, then start the stack with it (Command.md section 16, step 1).\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the planner's $WB_SIM_PROFILE block ($_pp_out) -- start the stack with WB_SIM_PROFILE=$WB_SIM_PROFILE first.\033[0m"
  fi
fi

case "${PNP_FEEDBACK:-fused}" in
  fused) LAUNCHER="$FUSED_LAUNCHER" ;;
  raw)   LAUNCHER="$FOUR_D_LAUNCHER" ;;
  *) echo "ERROR: PNP_FEEDBACK must be fused or raw (got '${PNP_FEEDBACK}')" >&2; exit 1 ;;
esac
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

if [[ "$PEGASUS_PNP_TASK" == drop ]]; then
  _PNP_DESC="PICK-AND-DROP: the real payload (Box_Assembly; ${PEGASUS_PNP_PAYLOAD:-assembly}) on the PICK hat, pillar tops ${PEGASUS_PNP_PICK_PILLAR_HEIGHT:-0.77} / ${PEGASUS_PNP_PLACE_PILLAR_HEIGHT:-0.77} m, dropped into the plastic tray (the bin cut to 35 mm walls, 371 x 290 mm, turned 90 deg) on the PLACE hat"
else
  _PNP_DESC="the ORIGINAL PICK-AND-PLACE (the paper's comparison): the one-piece basket (${PEGASUS_PNP_PAYLOAD:-box}) on the PICK hat, pillar tops ${PEGASUS_PNP_PICK_PILLAR_HEIGHT:-1.0} / ${PEGASUS_PNP_PLACE_PILLAR_HEIGHT:-1.0} m, set down on the PLACE hat"
fi
echo -e "\033[1;35mPICK-AND-PLACE SCENE ($AM_ISAAC_SCENE_LABEL, WB_SIM_PROFILE=$WB_SIM_PROFILE, ${PNP_FEEDBACK:-fused} feedback): $_PNP_DESC; field 4.5 x 4.2 m, vehicle at ${PEGASUS_PNP_SPAWN_XY:-0.10,-0.07} facing +x, PICK at (1.0, 1.0), PLACE at (-1.0, -1.0), payload ${PEGASUS_PNP_PAYLOAD_MASS:-$([[ "$PEGASUS_PNP_TASK" == drop ]] && echo 0.18 || echo 0.2)} kg. Mocap: /obj_0/mocap = the payload (CG + yaw), /drop_0/mocap = PLACE pillar top.\033[0m"
exec "$LAUNCHER" "$@"
