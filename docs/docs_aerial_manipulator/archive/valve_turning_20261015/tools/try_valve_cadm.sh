#!/usr/bin/env bash
# One valve-turning flight on the COORDINATED-ADMITTANCE comparison rig (Liu et al. CEP 2024),
# headless (2026-10-17): the twin of try_valve.sh / the box's run_pl_cadm.sh. Regenerates the
# whole-body valve yaml and the CA valve profile from it (the task block is shared), applies
# KEY=VAL overrides to the CA yaml (push_pull_* -> its planner block; cadm_* / admittance_* /
# mpc_* -> its controller + onboard-planner keys), flies the mission with the box push's
# pl_mission.py on that rig's namespace, restores the yaml.
#   try_valve_cadm.sh <tag> [key=val ...]      (clean slate first, in a SEPARATE call)
# Scene knobs (PEGASUS_VALVE_*) pass through the environment and are recorded in runs/<tag>.args.
set -o pipefail
TAG="${1:?tag}"; shift
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PEG="$HOME/fsc_PegasusSimulator"
CAGEN="$PEG/docs/docs_aerial_manipulator/archive/push_pull_coordinated_admittance_20261010/tools/make_coordinated_yaml.py"
PL="$PEG/docs/docs_aerial_manipulator/archive/push_pull_20261003/tools"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
STACK="$STACKS/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh"
Y="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650_sim_valve_turning.yaml"
DA="fsc_autopilot_ros2/coordinated_admittance_direct_actuation"
RUNS="${PL_RUNS_DIR:-$C/runs}"; mkdir -p "$RUNS"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
/usr/bin/python3 "$C/tools/make_valve_turning_yaml.py" > /dev/null
/usr/bin/python3 "$CAGEN" --profile valve_turning > /dev/null
for kv in "$@"; do
  k=${kv%%=*}; v=${kv#*=}
  grep -q "^    $k:" "$Y" || { echo "no key $k in $(basename "$Y")"; exit 1; }
  sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" "$Y"; echo "  $k -> $v"
done
cp "$Y" "$RUNS/$TAG.yaml"
REL=0; [[ " ${PL_DRIVER_ARGS:-} " == *" --release-at-open "* ]] && REL=1
{ printf '%s\n' "$@"; echo "FRICTION=${PEGASUS_VALVE_FRICTION:-0.048}"; echo "DAMPING=${PEGASUS_VALVE_DAMPING:-0.0}";
  echo "ROTOR_MASS=${PEGASUS_VALVE_ROTOR_MASS:-0.204}"; echo "RELEASE=$REL"; echo "DRIVER_ARGS=${PL_DRIVER_ARGS:-}"; } > "$RUNS/$TAG.args"

(WB_SIM_PROFILE=valve_turning setsid nohup bash "$STACK" shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
sleep 20
echo "[$(date +%T)] waiting for the planner's valve-turning block"
for i in $(seq 60); do
  off=$(ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner push_pull_turn_deg 2>/dev/null \
        | sed -nE 's/.*values? (is|are): *(.*)$/\2/p')
  [[ -n "$off" ]] && break; sleep 2
done
[[ -n "$off" ]] || { echo "the planner has no push_pull_turn_deg -- old binary? rebuild fsc_trajectory_planner"; exit 1; }
echo "[$(date +%T)] planner up, turn $off deg"
(setsid nohup env PEGASUS_HEADLESS=1 WB_SIM_PROFILE=valve_turning \
   "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_coordinated_admittance_valve_turning_sitl.sh" --in-terminal shiqi_machine \
   > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the valve (Isaac)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] valve up; letting PX4 / the arm stack settle"
sleep 45
echo "[$(date +%T)] flying"
/usr/bin/python3 "$PL/pl_mission.py" --da "$DA" --dbg-topic coordinated_control_debug --dbg-len 56 \
  --hover-dwell "${PL_HOVER_DWELL:-15.0}" --grip-dwell-in "${PL_GRIP_DWELL_IN:-0.3}" --grip-tol "${PL_GRIP_TOL:-0.025}" \
  --grip-axis-tol "${PL_GRIP_AXIS_TOL:-0.004}" ${PL_GRIP_LIVE_BOX:---grip-live-box} --out "$RUNS/$TAG.npz" ${PL_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
echo "[$(date +%T)] done"
/usr/bin/python3 "$CAGEN" --profile valve_turning > /dev/null      # restore the generated yaml
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$RUNS/$TAG.log" | head -1 | cut -c1-150)"
