#!/usr/bin/env bash
# PICK-AND-PLACE SCENE (2026-10-01) for the whole-body L1 4-D rig.
#
# A thin wrapper: it swaps the Isaac app for
# application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py
# (06's free-flight plant + the field, the two 1 m pillars and the 200 g
# payload, published as the mocap bodies obj_0 and drop_0) and hands over to
# the 4-D launcher. Everything else -- the plant knobs from the paired yaml,
# the 4-D node check, the torque-mode arm stack, both ground stations, the
# gamepad -- is that launcher's, unchanged.
#
#   PNP_FEEDBACK=fused (default)  EKF2-fused odometry, the hardware path:
#       WB_SIM_PROFILE=pick_and_place start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh first
#   PNP_FEEDBACK=raw              raw mocap odometry:
#       WB_SIM_PROFILE=pick_and_place start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh first
#
# Usage: start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
FOUR_D_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
FUSED_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_fused_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py"
export AM_ISAAC_SCENE_LABEL="AM-T650-WB-L1-4D-PNP"
# The payload is obj_0 in this scene; the EE marker cube would publish it too.
export PEGASUS_EE_MARKER_CUBE=0
# The paired config: the mirror plant + controller with the tuned pick-and-place
# planner block (..._sim_pick_and_place.yaml). The STACK must be started with the
# same profile -- this launcher checks the running planner below.
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-pick_and_place}"

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }

# The scene's knobs (07's PEGASUS_PNP_*) must reach the Isaac pane, which tmux
# starts from its SERVER's environment -- and the server is already up (the
# stack started it). Push the set ones into the global environment and clear
# the unset ones, so a knob from an earlier run cannot leak into this one.
# (list-sessions, not `tmux info`: info needs an attached client, and a
# background launch has none -- the knobs then silently never arrive)
if tmux list-sessions >/dev/null 2>&1; then
  for v in PEGASUS_PNP_PAYLOAD PEGASUS_PNP_PAYLOAD_USD PEGASUS_PNP_GRIP_FRICTION PEGASUS_PNP_PAYLOAD_MASS PEGASUS_PNP_PAYLOAD_YAW_DEG PEGASUS_PNP_HANDLE_THICKNESS PEGASUS_PNP_PLATFORM PEGASUS_PNP_HAT_USD PEGASUS_PNP_CAP_DIAMETER PEGASUS_PNP_GRIP_TORQUE PEGASUS_PNP_GRASP_TEST PEGASUS_PNP_WAYPOINTS PEGASUS_PNP_SPAWN_XY PEGASUS_PNP_SPAWN_YAW_DEG; do
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
PP_PLANNER_YAML="${WB_SIM_YAML:-${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml}"
if [[ "${PP_CHECK:-1}" != 0 && ( "$WB_SIM_PROFILE" == pick_and_place || "$WB_SIM_PROFILE" == pick_place ) ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/pick_place_planner_check.sh"
  set +e; _pp_out=$(pick_place_planner_check "$PP_PLANNER_YAML" "$PP_PLANNER_YAML"); _pp_rc=$?; set -e
  if [[ $_pp_rc == 0 ]]; then
    echo -e "\033[1;32mplanner: pick-and-place block loaded (planner poses and waypoints + controller match the pick-and-place yamls)\033[0m"
  elif [[ $_pp_rc == 1 ]]; then
    echo -e "\033[1;31m$_pp_out\033[0m"
    echo -e "\033[1;31mREFUSED: the running planner does not carry the pick-and-place block -- the stack was started without\033[0m"
    echo -e "\033[1;31mWB_SIM_PROFILE=pick_place. Clean slate, then start the stack with it (Command.md section 16, step 1).\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the planner's pick-and-place block ($_pp_out) -- start the stack with WB_SIM_PROFILE=pick_place first.\033[0m"
  fi
fi

case "${PNP_FEEDBACK:-fused}" in
  fused) LAUNCHER="$FUSED_LAUNCHER" ;;
  raw)   LAUNCHER="$FOUR_D_LAUNCHER" ;;
  *) echo "ERROR: PNP_FEEDBACK must be fused or raw (got '${PNP_FEEDBACK}')" >&2; exit 1 ;;
esac
[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }

echo -e "\033[1;35mPICK-AND-PLACE SCENE ($AM_ISAAC_SCENE_LABEL, ${PNP_FEEDBACK:-fused} feedback): field 4.5 x 4.2 m (x by y), vehicle at ${PEGASUS_PNP_SPAWN_XY:-0.10,-0.07} facing +x, pillars at (1.0, 1.0) PICK and (-1.0, -1.0) PLACE, ${PEGASUS_PNP_PAYLOAD_MASS:-0.2} kg payload (${PEGASUS_PNP_PAYLOAD:-box}: box = the CAD basket + hanger, hook grasp; plate = the old box + 20 mm handle) on the PICK pillar, ${PEGASUS_PNP_PLATFORM:-hat} on both pillars (hat = the printed Top_Hat, its platform 8 mm above the 1 m pillar top; cap = the old 160 mm disc). Mocap: /obj_0/mocap = the payload box (CG + yaw), /drop_0/mocap = PLACE pillar top.\033[0m"
exec "$LAUNCHER" "$@"
