#!/usr/bin/env bash
# One headless PICK-AND-DROP flight for the three-controller comparison (2026-10-26):
#
#   run_pnd.sh wb|geo|mod <tag>        (after a clean slate in a SEPARATE process:
#                                       scripts/kill_stale_sim_processes.sh -y)
#
# Same scene, task, plant and launch condition on every rig:
#   * 07 with PEGASUS_PNP_TASK=drop: the Box_Assembly payload clamped by its fin, 0.77 m
#     pillars, the 35 mm tray turned 90 deg on the PLACE hat; payload PNP_MASS kg (default 0.200);
#   * the planner block of the whole-body ..._sim_pick_and_drop.yaml (every rig's file carries it
#     verbatim: the MAC yaml is generated from it, the geometric rig reads it via WB_PLANNER_YAML);
#   * PEGASUS_ARM_SPIN_DRAIN=1 (2026-10-26: the arm bridge applies the newest effort/position
#     command each physics step; archive/pick_place_mac_20261026);
#   * headless Isaac at RTF 1, raw mocap, the mirror plant.
# Controllers:
#   wb   whole-body 4-D L1: ..._whole_body_l1_4d_..._sim_pick_and_drop.yaml
#   geo  geometric + L1 (decoupled, position-mode arm): ..._geometric_l1_..._sim_pick_and_place.yaml
#        (its pick-and-place gains kp/kv 15/10, kp_z/kv_z 40/18)
#   mod  modular adaptive (MAC): ..._modular_adaptive_..._sim_pick_and_drop.yaml (generated)
# The PEGASUS_* scene knobs are EXPORTED before the controller stack starts: the stack starts the
# tmux server, and the Isaac pane only sees the server's environment (the MAC launcher has no
# pick-and-place wrapper to push them). Read the scene banner in <tag>.isaac.log.
# Writes ../runs/<tag>.{npz,log,stack.log,scene.log,isaac.log}. Leaves the simulation running.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
RIG="${1:?rig: wb, geo or mod}"; TAG="${2:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="${PND_RUNS_DIR:-$HERE/../runs}"; mkdir -p "$RUNS"
PEG="$HOME/fsc_PegasusSimulator"
CFG="$HOME/ros2_ws/src/fsc_autopilot_ros2/config"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
WB_PND="$CFG/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_drop.yaml"
GEO_CTRL="$CFG/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml"
MOD_PND="$CFG/params_single_aerial_manipulator_modular_adaptive_direct_actuation_t650_sim_pick_and_drop.yaml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
export PEGASUS_PNP_TASK=drop
export PEGASUS_PNP_PAYLOAD_MASS="${PNP_MASS:-0.200}"
export PEGASUS_ARM_SPIN_DRAIN="${PND_SPIN_DRAIN:-1}"

case "$RIG" in
  wb)
    (WB_SIM_PROFILE=pick_and_drop setsid nohup bash "$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh" \
        shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
    SCENE=(env PEGASUS_HEADLESS=1 PNP_FEEDBACK=raw WB_SIM_PROFILE=pick_and_drop
           "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh" shiqi_machine) ;;
  geo)
    (WB_SIM_PROFILE=pick_and_drop WB_SIM_YAML="$GEO_CTRL" WB_PLANNER_YAML="$WB_PND" setsid nohup bash \
        "$STACKS/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh" \
        shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
    SCENE=(env PEGASUS_HEADLESS=1 WB_SIM_PROFILE=pick_and_drop WB_SIM_YAML="$GEO_CTRL" WB_PLANNER_YAML="$WB_PND"
           "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh" shiqi_machine) ;;
  mod)
    (WB_SIM_PROFILE=pick_and_place WB_SIM_YAML="$MOD_PND" setsid nohup bash \
        "$STACKS/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh" \
        shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
    SCENE=(env PEGASUS_HEADLESS=1 WB_SIM_PROFILE=pick_and_place WB_SIM_YAML="$MOD_PND"
           AM_ISAAC_SCENE_SCRIPT="$PEG/application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py"
           AM_ISAAC_SCENE_LABEL="AM-T650-MODULAR-PND" PEGASUS_EE_MARKER_CUBE=0
           "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh" shiqi_machine) ;;
  *) echo "rig must be wb, geo or mod" >&2; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"

# the stacks refuse to start beside a running planner and check with pgrep -f: a poll naming
# the planner while that check runs would match it -- let the stack get past its guards first
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
echo "[$(date +%T)] flying"
/usr/bin/python3 "$PEG/docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/tools/pnp_mission_v2.py" \
    --rig "$RIG" --out "$RUNS/$TAG.npz" ${PNP_DRIVER_ARGS:---drop-place --grip-via-action --grip-axis-tol 0.003 --timetable} 2>&1 | tee "$RUNS/$TAG.log"
dump_isaac
echo "[$(date +%T)] done"
