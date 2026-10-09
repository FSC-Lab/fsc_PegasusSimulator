#!/usr/bin/env bash
# One AUTONOMOUS pick-and-place flight in Isaac, headless, on either rig -- the
# 2026-10-08 copy for the hat platform (the scene default since that day)
# (2026-10-07 copy: PNP_PROBE_ONLY=1 stops after bring-up; the Isaac pane's
# output is saved to runs/<tag>.isaac.log):
#
#   run_pnp.sh wb|geo <tag>        (after a clean slate in a SEPARATE call:
#                                   scripts/kill_stale_sim_processes.sh -y)
#
#   wb   the whole-body 4-D L1 rig: WB_SIM_PROFILE=pick_and_place (the planner
#        block and the controller from ..._l1_4d_..._sim_pick_and_place.yaml)
#   geo  the decoupled geometric + L1 rig: the controller from
#        ..._geometric_l1_..._sim_pick_and_place.yaml, the planner block from the
#        whole-body pick-and-place yaml (WB_PLANNER_YAML) -- the same task
#
# PNP_PAYLOAD_MASS=<kg> overrides the scene's 200 g payload (env); PNP_GRIP_FRICTION="s,d" its friction.
# PNP_DRIVER_ARGS="..." is passed to pnp_mission_v2.py (e.g. "--clamp-hold 4").
# Both fly RAW mocap odometry (the decoupled rig has no fused sim stack): one
# launch condition. Brings the stack and the scene up in the background,
# waits for the planner, the payload and PX4, flies pnp_mission_v2.py and
# writes ../runs/<tag>.{npz,log} plus the stack / scene logs. Leaves the
# simulation running: the next clean slate (or the campaign's end) closes it.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
RIG="${1:?rig: wb, geo or mod}"; TAG="${2:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$HERE/../runs"; mkdir -p "$RUNS"
PEG="$HOME/fsc_PegasusSimulator"
CFG="$HOME/ros2_ws/src/fsc_autopilot_ros2/config"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
WB_PNP="$CFG/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml"
GEO_PNP="$CFG/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"

case "$RIG" in
  wb)
    (WB_SIM_PROFILE=pick_and_place setsid nohup bash "$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh" \
        shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
    SCENE=(env PEGASUS_HEADLESS=1 PNP_FEEDBACK=raw WB_SIM_PROFILE=pick_and_place ${PNP_PAYLOAD_MASS:+PEGASUS_PNP_PAYLOAD_MASS=$PNP_PAYLOAD_MASS} ${PNP_GRIP_FRICTION:+PEGASUS_PNP_GRIP_FRICTION=$PNP_GRIP_FRICTION}
           "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh" shiqi_machine) ;;
  geo)
    (WB_SIM_YAML="$GEO_PNP" WB_PLANNER_YAML="$WB_PNP" setsid nohup bash \
        "$STACKS/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh" \
        shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
    SCENE=(env PEGASUS_HEADLESS=1 WB_SIM_YAML="$GEO_PNP" ${PNP_PAYLOAD_MASS:+PEGASUS_PNP_PAYLOAD_MASS=$PNP_PAYLOAD_MASS} ${PNP_GRIP_FRICTION:+PEGASUS_PNP_GRIP_FRICTION=$PNP_GRIP_FRICTION}
           "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh" shiqi_machine) ;;
  mod)
    (WB_SIM_PROFILE=pick_and_place setsid nohup bash "$STACKS/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh" \
        shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
    SCENE=(env PEGASUS_HEADLESS=1 WB_SIM_PROFILE=pick_and_place AM_ISAAC_SCENE_SCRIPT="$PEG/application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py" AM_ISAAC_SCENE_LABEL="AM-T650-MODULAR-PNP" PEGASUS_EE_MARKER_CUBE=0 ${PNP_PAYLOAD_MASS:+PEGASUS_PNP_PAYLOAD_MASS=$PNP_PAYLOAD_MASS} ${PNP_GRIP_FRICTION:+PEGASUS_PNP_GRIP_FRICTION=$PNP_GRIP_FRICTION}
           "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh" shiqi_machine) ;;
  *) echo "rig must be wb, geo or mod" >&2; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"

# the stacks refuse to start beside a running planner and check with pgrep -f:
# a poll naming the planner while that check runs would match it -- let the
# stack get past its guards first
sleep 20
echo "[$(date +%T)] waiting for the planner"
for i in $(seq 60); do
  dz=$(ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner pick_place_approach_dz 2>/dev/null \
       | sed -nE 's/.*value is: *([-0-9.eE+]+).*/\1/p')
  [[ -n "$dz" ]] && break; sleep 2
done
echo "[$(date +%T)] planner up, safety margin $dz m"
(setsid nohup "${SCENE[@]}" > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the payload (Isaac)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] payload up; letting PX4 / the arm stack settle"
sleep 45
dump_isaac() {   # the Isaac pane's own output (probes, banners) lives only in tmux
  for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
    tmux capture-pane -J -p -S - -t "$p" 2>/dev/null
  done | sed 's/\x1b\[[0-9;]*m//g' | grep -E "AM-T650|Traceback|Error" > "$RUNS/$TAG.isaac.log" || true
}
if [[ "${PNP_PROBE_ONLY:-0}" == 1 ]]; then
  dump_isaac; echo "[$(date +%T)] probe only: $RUNS/$TAG.isaac.log"; exit 0
fi
echo "[$(date +%T)] flying"
/usr/bin/python3 "$HERE/../../pick_place_controllers_20261003/tools/pnp_mission_v2.py" --rig "$RIG" --out "$RUNS/$TAG.npz" ${PNP_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
dump_isaac
echo "[$(date +%T)] done"
