#!/usr/bin/env bash
# One AUTONOMOUS mail-delivery flight in Isaac, headless, whole-body 4-D L1 rig (2026-10-23).
# A copy of lift_lurch_gains_20261016/tools/run_pnp.sh for WB_SIM_PROFILE=mail_delivery: the
# stack on ..._sim_mail_delivery.yaml, the 07 mail-delivery scene through the pick-and-place
# wrapper, and pnp_mission_v2.py --mail (the arm GS Mail Envelope tab's sequence).
#
#   run_mail.sh <tag>                (after a clean slate in a SEPARATE call:
#                                     scripts/kill_stale_sim_processes.sh -y)
#
# MAIL_FEEDBACK=raw (default) | fused   the odometry the law flies (fused = the hardware path)
# MAIL_SCENE_ENV="K=V ..."              any scene knob (07 mail-delivery's PEGASUS_MAIL_*)
# MAIL_DRIVER_ARGS="..."                extra pnp_mission_v2.py arguments
# MAIL_PROBE_ONLY=1                     stop after bring-up (the Isaac pane's banners and probes)
# Writes ../runs/<tag>.{npz,log,stack.log,scene.log,isaac.log}; leaves the simulation running.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
TAG="${1:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$HERE/../runs"; mkdir -p "$RUNS"
PEG="$HOME/fsc_PegasusSimulator"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
FB="${MAIL_FEEDBACK:-raw}"
case "$FB" in
  raw)   STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh" ;;
  fused) STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh" ;;
  *) echo "MAIL_FEEDBACK must be raw or fused" >&2; exit 2 ;;
esac
(env WB_SIM_PROFILE=mail_delivery setsid nohup bash "$STACK" shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
SCENE=(env PEGASUS_HEADLESS=1 PNP_FEEDBACK=$FB WB_SIM_PROFILE=mail_delivery ${MAIL_SCENE_ENV:-}
       "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh" shiqi_machine)

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"

# the stacks refuse to start beside a running planner and check with pgrep -f
# on its NODE name: a poll whose command line names the node would match it (the
# fused stack reaches that check later than 20 s -- mail4's first try). So wait on
# the planner's pick-and-place status TOPIC, whose name does not contain it.
sleep 20
echo "[$(date +%T)] waiting for the planner"
for i in $(seq 60); do
  st=$(timeout 6 ros2 topic echo --no-daemon --once /uav_0/whole_body_planner/pick_place/status 2>/dev/null | grep -m1 '^data:')
  [[ -n "$st" ]] && break; sleep 2
done
echo "[$(date +%T)] planner up: $st"
(setsid nohup "${SCENE[@]}" > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the envelope (Isaac)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] envelope up; letting PX4 / the arm stack settle"
sleep 45
dump_isaac() {   # the Isaac pane's own output (probes, banners, the envelope's state) lives only in tmux
  for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
    tmux capture-pane -J -p -S - -t "$p" 2>/dev/null
  done | sed 's/\x1b\[[0-9;]*m//g' | grep -E "AM-T650|Traceback|Error" > "$RUNS/$TAG.isaac.log" || true
}
if [[ "${MAIL_PROBE_ONLY:-0}" == 1 ]]; then
  dump_isaac; echo "[$(date +%T)] probe only: $RUNS/$TAG.isaac.log"; exit 0
fi
echo "[$(date +%T)] flying"
/usr/bin/python3 "$HERE/../../pick_place_controllers_20261003/tools/pnp_mission_v2.py" --rig wb --out "$RUNS/$TAG.npz" \
  --mail --grip-via-action --grip-axis-tol 0.008 ${MAIL_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
dump_isaac
echo "[$(date +%T)] done"
