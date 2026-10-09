#!/usr/bin/env bash
# PICK-AND-PLACE SCENE (2026-10-03) for the DECOUPLED geometric + L1 rig: the
# same Isaac scene, payload, pillars and planned task as the whole-body 4-D
# pick-and-place (start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh),
# only the controller differs.
#
# A thin wrapper: it swaps the Isaac app for 07 (06's plant + the field, the
# capped pillars and the 200 g payload) and hands over to the decoupled
# launcher, whose arm runs in POSITION mode (the position-mode ros2_control
# stack + arm_planner). The plant knobs are read from the decoupled
# pick-and-place controller yaml (WB_SIM_YAML), whose plant section is the
# whole-body mirror's -- the same plant.
#
# Start the external stack first, with the SAME task (the profile selects this
# rig's ..._sim_pick_and_place.yaml controller and the whole-body planner block):
#   WB_SIM_PROFILE=pick_and_place \
#     fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh <config> uav_0
#
# Usage: start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh [--in-terminal] <config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh"

export AM_ISAAC_SCENE_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py"
export AM_ISAAC_SCENE_LABEL="AM-T650-GEO-L1-PNP"
# The payload is obj_0 in this scene; the EE marker cube would publish it too.
export PEGASUS_EE_MARKER_CUBE=0
FSC_AUTOPILOT_CONFIG="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config"
export WB_SIM_YAML="${WB_SIM_YAML:-$FSC_AUTOPILOT_CONFIG/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml}"

[[ -f "$AM_ISAAC_SCENE_SCRIPT" ]] || { echo "ERROR: missing $AM_ISAAC_SCENE_SCRIPT" >&2; exit 1; }
[[ -r "$WB_SIM_YAML" ]] || { echo "ERROR: controller yaml not readable: $WB_SIM_YAML" >&2; exit 1; }

# The scene's knobs (07's PEGASUS_PNP_*) must reach the Isaac pane, which tmux
# starts from its SERVER's environment (the stack started the server): push the
# set ones, clear the unset ones -- the whole-body wrapper's logic.
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

# The planner must run the pick-and-place block (the stack's WB_PLANNER_YAML).
if command -v ros2 >/dev/null 2>&1; then
  _dz=$(timeout 15 ros2 param get /uav_0/whole_body_trajectory_planner pick_place_approach_dz --no-daemon 2>/dev/null \
        | sed -nE 's/.*value is: *([-0-9.eE+]+).*/\1/p' || true)
  if [[ -z "$_dz" ]]; then
    echo -e "\033[1;33mWARNING: could not read pick_place_approach_dz off /uav_0/whole_body_trajectory_planner.\033[0m"
  else
    echo -e "\033[1;32mplanner: safety margin (approach_dz) $_dz m -- start the stack with WB_PLANNER_YAML=..._sim_pick_and_place.yaml for the pick-and-place block\033[0m"
  fi
fi

[[ -x "$LAUNCHER" ]] || { echo "ERROR: missing executable $LAUNCHER" >&2; exit 1; }
echo -e "\033[1;35mPICK-AND-PLACE SCENE ($AM_ISAAC_SCENE_LABEL): the DECOUPLED geometric+L1 controller, position-mode arm; the whole-body pick-and-place task and plant.\033[0m"
# The planner must carry the pick-and-place block (the stack's WB_PLANNER_YAML):
# without it it flies its defaults -- pick / place pose [0, 0, 0, 0], carry =
# the folded home pose, no clockwise turns. Refuse on a mismatch; PP_CHECK=0 skips.
PP_PLANNER_YAML="${WB_PLANNER_YAML:-$FSC_AUTOPILOT_CONFIG/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml}"
if [[ "${PP_CHECK:-1}" != 0 ]] && command -v ros2 >/dev/null 2>&1; then
  source "$SCRIPT_DIR/lib/pick_place_planner_check.sh"
  set +e; _pp_out=$(pick_place_planner_check "$PP_PLANNER_YAML" "$WB_SIM_YAML"); _pp_rc=$?; set -e
  if [[ $_pp_rc == 0 ]]; then
    echo -e "\033[1;32mplanner: pick-and-place block loaded (planner poses and waypoints + controller match the pick-and-place yamls)\033[0m"
  elif [[ $_pp_rc == 1 ]]; then
    echo -e "\033[1;31m$_pp_out\033[0m"
    echo -e "\033[1;31mREFUSED: the running planner does not carry the pick-and-place block -- clean slate, then start the\033[0m"
    echo -e "\033[1;31mdecoupled stack with WB_SIM_PROFILE=pick_and_place (Command.md section 16).\033[0m"
    exit 1
  else
    echo -e "\033[1;33mWARNING: could not verify the planner's pick-and-place block ($_pp_out).\033[0m"
  fi
fi

exec "$LAUNCHER" "$@"
