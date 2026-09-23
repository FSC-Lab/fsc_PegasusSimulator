#!/usr/bin/env bash
# One whole-body observer data point, clean slate to npz.
#
#   wb_l1_tune_cycle.sh <gmo|l1|l1_4d|l1_4d_fused> <run-tag> [machine-config]
#
# l1_4d_fused (2026-09-21) is l1_4d with the controller's position/velocity
# feedback taken from PX4 EKF2's FUSED odometry (indoor_state_estimator_node)
# instead of the raw mocap message -- the simulation proof of the hardware
# ..._stack_t650_aerial_manipulator_fused.sh workflow. Same node, same yaml.
#
# l1_4d (2026-09-16) is the L1 observer with the working note's
# FOUR-DIMENSIONAL attribution: same executable as l1, its own stack script
# and yaml (..._l1_4d_..._sim.yaml); the Pegasus launcher reads wb_l1_four_d
# off the running node to confirm which design is flying.
#
# MISSION (2026-09-17): WB_L1_MISSION picks what is flown after the bring-up.
#   standard  (default) wb_l1_campaign_driver.py -- the 7.15.5 steps + compatible
#             trajectories, full wb_control_debug recorded
#   ee_circle / ee_figure8   fsc_trajectory_planner's END-EFFECTOR TRAJECTORY
#             mode, flown by that package's installed ee_trajectory_sim_driver.py
#             (select -> time scale -> go_to_start -> start, exactly what the arm
#             GS's "EE trajectory" tab publishes). WB_L1_EE_SCALE = fraction of
#             the planner's s_max (default 0.8). Records odometry, the EE
#             reference and the measured EE, not the control debug. The planner
#             package's own ee_trajectory_sim_cycle.sh hard-codes the 6-D stack;
#             this is how the GMO / 6-D / 4-D rigs fly the same shape.
#
# All cases fly the SAME plant through the SAME mission with the SAME driver;
# only the controller node and its yaml differ, which is the whole point. The
# `_sim` yamls are used (not the comparison ones), because those are the pair
# that carries the deliberate deviations the campaign is about: a +15%
# allocator kf (thrust loss), plant mass and inertia x1.10 with a 10/10/5 mm
# CoM shift (model uncertainty), and the MN4010 rotor lag (motor delay).
#
# Params are read at controller STARTUP and PX4 never disarms this rig, so
# every data point needs a full relaunch -- that is what this script is for.
#
# TO SWEEP A GAIN: edit the wb_l1_* block of
# params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml
# between runs, or point WB_L1_YAML at a variant. There is no on-set-parameters
# callback in this node, so `ros2 param set` changes what `param get` reports
# while the controller keeps flying the launch-time value.
#
# NOTE the launcher guards use `pgrep -f` on the controller node name, which
# ALSO matches any shell whose command line contains it. Everything here runs
# from a script file, so the command line is this script's name -- never inline
# these commands in a terminal.
set -uo pipefail

WHICH="${1:?usage: wb_l1_tune_cycle.sh <gmo|l1|l1_4d|l1_4d_fused> <run-tag> [machine-config]}"
TAG="${2:?usage: wb_l1_tune_cycle.sh <gmo|l1|l1_4d|l1_4d_fused> <run-tag> [machine-config]}"
CFG="${3:-shiqi_machine}"
case "$WHICH" in gmo|l1|l1_4d|l1_4d_fused) ;; *) echo "first arg must be gmo, l1, l1_4d or l1_4d_fused"; exit 2;; esac
MISSION="${WB_L1_MISSION:-standard}"
case "$MISSION" in standard|ee_circle|ee_figure8) ;; *)
  echo "WB_L1_MISSION must be standard, ee_circle or ee_figure8"; exit 2;; esac

PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
# shellcheck source=/dev/null
source "$PEG/scripts/config/${CFG}.conf"
AUT="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2"

if [[ "$WHICH" == l1 ]]; then
  STACK="$AUT/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh"
  SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh"
  NODE="autopilot_whole_body_l1_direct_actuation_node"
elif [[ "$WHICH" == l1_4d ]]; then
  STACK="$AUT/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
  SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
  NODE="autopilot_whole_body_l1_direct_actuation_node"
elif [[ "$WHICH" == l1_4d_fused ]]; then
  STACK="$AUT/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh"
  SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_fused_direct_actuation_sitl.sh"
  NODE="autopilot_whole_body_l1_direct_actuation_node"
else
  STACK="$AUT/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh"
  SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh"
  NODE="autopilot_whole_body_direct_actuation_node"
fi

OUT="${WB_L1_OUT:-$PEG/docs/docs_aerial_manipulator/l1_observer_20260906}"
LOGS="$OUT/logs"
mkdir -p "$OUT" "$LOGS"

# ROS ENVIRONMENT, EXPLICITLY. This machine can resolve fsc_autopilot_ros2_msgs
# to a stale workspace, which surfaces only as an ImportError in the driver
# AFTER the whole sim is up -- prove it here instead of three minutes later.
# ROS's own setup scripts read unset variables, so `set -u` stands down.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck source=/dev/null
source "${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/setup.bash"
set -u
/usr/bin/python3 -c \
  "from fsc_autopilot_ros2_msgs.msg import PositionControllerReference" 2>/dev/null || {
  echo "FAILED: fsc_autopilot_ros2_msgs is not the built one." >&2; exit 1; }
if [[ "$MISSION" != standard ]]; then
  # The stacks launch the planner from this install; without it there is no EE
  # trajectory mode to fly and the driver would only time out.
  EE_DRIVER="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/fsc_trajectory_planner/lib/fsc_trajectory_planner/ee_trajectory_sim_driver.py"
  [[ -f "$EE_DRIVER" ]] || {
    echo "FAILED: $EE_DRIVER not found -- build fsc_trajectory_planner first." >&2; exit 1; }
fi

# WB_L1_CLIENT_DDS_PROFILE (2026-09-21): a Fast DDS XML profile applied to THIS
# script's own ROS clients only -- the readiness echoes and the driver -- never
# to the stack or Pegasus it launches, so the system under test keeps its
# default transport. Needed on shiqi-desktop, where a participant started from
# a shell stopped receiving the ROS nodes' topics over shared memory (PX4's
# agent-bridged topics still arrived), which strands this script at "waiting
# for odometry". A UDP-only profile receives everything.
CLIENT_ENV=()
if [[ -n "${WB_L1_CLIENT_DDS_PROFILE:-}" ]]; then
  CLIENT_ENV=(env "FASTRTPS_DEFAULT_PROFILES_FILE=$WB_L1_CLIENT_DDS_PROFILE")
fi

echo "=== [$WHICH/$TAG] 0. clean slate ==="
"$AUT/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y  >/dev/null 2>&1
sleep 5

echo "=== [$WHICH/$TAG] 1. controller stack ==="
setsid nohup "$STACK" "$CFG" uav_0 > "$LOGS/stack_$TAG.log" 2>&1 < /dev/null &
for _ in $(seq 60); do pgrep -x MicroXRCEAgent >/dev/null && break; sleep 2; done
pgrep -x MicroXRCEAgent >/dev/null || {
  echo "FAILED: agent never came up"; tail -20 "$LOGS/stack_$TAG.log"; exit 1; }
for _ in $(seq 60); do pgrep -f "$NODE" >/dev/null && break; sleep 1; done
pgrep -f "$NODE" >/dev/null || {
  echo "FAILED: $NODE never came up"; tail -30 "$LOGS/stack_$TAG.log"; exit 1; }
sleep 8

echo "=== [$WHICH/$TAG] 2. Pegasus / PX4 / arm ==="
DISPLAY="${DISPLAY:-:0}" setsid nohup "$SITL" --in-terminal "$CFG" \
    > "$LOGS/pegasus_$TAG.log" 2>&1 < /dev/null &

# The readiness checks below run WITHOUT the ROS 2 daemon and with explicit
# types (2026-09-21): on shiqi-desktop the daemon wedged during stack bring-up
# (its graph query never returned), and a daemon-backed `ros2 topic echo`
# then timed out on every poll while the topic was publishing normally.
echo "=== [$WHICH/$TAG] 3. waiting for odometry ==="
ok=0
for _ in $(seq 120); do
  if "${CLIENT_ENV[@]}" timeout 5 ros2 topic echo --once --no-daemon /uav_0/state_estimator/local_position/odom nav_msgs/msg/Odometry \
       >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: no odometry"; tail -40 "$LOGS/pegasus_$TAG.log"; exit 1; }

# WAIT FOR THE EKF TO ALIGN, do not just sleep: the heading estimate needs tens
# of seconds of external-vision data after spawn, and arming before that gives
# "Preflight Fail: heading estimate not stable" -- which then looks exactly
# like a control failure and is not.
# GATE ON THE EKF FLAGS, NOT on vehicle_status.pre_flight_checks_pass: that
# field is false on this rig even while it is armed and flying.
echo "=== [$WHICH/$TAG] 3b. waiting for EKF yaw/EV alignment ==="
ok=0
for _ in $(seq 60); do
  f=$("${CLIENT_ENV[@]}" timeout 5 ros2 topic echo --once --no-daemon /uav_0/fmu/out/estimator_status_flags px4_msgs/msg/EstimatorStatusFlags 2>/dev/null)
  if grep -q "cs_yaw_align: true" <<<"$f" && grep -q "cs_ev_pos: true" <<<"$f" \
     && grep -q "cs_ev_yaw: true" <<<"$f"; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: EKF never aligned"; exit 1; }

# The arm ros2_control stack activates a few seconds after Isaac starts
# reporting; the DIRECT entry gate checks the arm against its own reference, so
# waiting for joint states here avoids a refusal that has nothing to do with
# the observer.
echo "=== [$WHICH/$TAG] 3c. waiting for the arm ==="
ok=0
for _ in $(seq 60); do
  if "${CLIENT_ENV[@]}" timeout 5 ros2 topic echo --once --no-daemon /uav_0/fsc_open_manipulator/joint_states sensor_msgs/msg/JointState \
       >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || echo "WARNING: no arm joint states -- DIRECT entry may refuse"
sleep 10

echo "=== [$WHICH/$TAG] 4. flying ($MISSION) ==="
if [[ "$MISSION" == standard ]]; then
  "${CLIENT_ENV[@]}" /usr/bin/python3 "$PEG/application/robotic_arm/utils/wb_l1_campaign_driver.py" \
      --out "$OUT/${WHICH}_${TAG}.npz" ${WB_L1_DRIVER_ARGS:-} \
      > "$LOGS/${WHICH}_${TAG}.log" 2>&1
  rc=$?
else
  "${CLIENT_ENV[@]}" /usr/bin/python3 "$EE_DRIVER" --shape "${MISSION#ee_}" --scale "${WB_L1_EE_SCALE:-0.8}" \
      --out "$OUT/${WHICH}_${TAG}.npz" ${WB_L1_DRIVER_ARGS:-} \
      > "$LOGS/${WHICH}_${TAG}.log" 2>&1
  rc=$?
fi

echo "=== [$WHICH/$TAG] done (driver rc=$rc) ==="
tail -8 "$LOGS/${WHICH}_${TAG}.log"
exit $rc
