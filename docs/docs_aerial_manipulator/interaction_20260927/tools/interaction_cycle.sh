#!/usr/bin/env bash
# One Isaac interaction flight, clean slate to npz (2026-09-27).
#
#   interaction_cycle.sh <mirror|thr2|contact> <run-tag> --seg t0:hold:dir:F [--seg ...]
#
# mirror  = the shipped 4-D _sim.yaml (H1b), untouched
# thr2    = ..._sim_interaction_thr2.yaml    (reading omega_x 2, chi threshold 2 N)
# contact = ..._sim_interaction_contact.yaml (reading omega_x 2, chi = contact all DIRECT)
#
# Plant: 07 (06 + the EE force injector) via the interaction launcher; the
# controller stack is the 4-D one, pointed at the yaml by WB_SIM_YAML (both
# the stack and the Pegasus plant library honour it). Mission: the campaign
# driver's hover-only soak; the force driver streams the profile in SIM time
# from the DIRECT edge and records the controller's reading of it.
# Mirrors wb_l1_tune_cycle.sh (same gates, same traps) -- that script
# hard-codes the 06 launcher, so this is a copy, not an edit.
set -uo pipefail
YT="${1:?usage: interaction_cycle.sh <mirror|thr2|contact> <tag> --seg ...}"
TAG="${2:?tag}"; shift 2
SEGS=("$@")
CFG="${INTERACTION_CFG:-shiqi_machine}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../../.." && pwd)"
# shellcheck source=/dev/null
source "$PEG/scripts/config/${CFG}.conf"
AUT="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2"
CFGD="$AUT/config"
case "$YT" in
  mirror)  YAML="$CFGD/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml" ;;
  thr2|contact) YAML="$CFGD/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_interaction_${YT}.yaml" ;;
  *) echo "yaml tag must be mirror, thr2 or contact"; exit 2 ;;
esac
[[ -r "$YAML" ]] || { echo "missing $YAML"; exit 2; }
export WB_SIM_YAML="$YAML"
STACK="$AUT/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_interaction_sitl.sh"
NODE="autopilot_whole_body_l1_direct_actuation_node"
OUT="$HERE/../data"; LOGS="$HERE/../logs"; mkdir -p "$OUT" "$LOGS"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/setup.bash"
set -u
CLIENT_ENV=(env "FASTRTPS_DEFAULT_PROFILES_FILE=$HERE/fastdds_udp_only.xml")

# profile length in SIM seconds -> the soak the hover driver must hold (wall)
TEND=$(/usr/bin/python3 -c "
import sys; a=sys.argv[1:]; s=[a[i+1] for i in range(len(a)) if a[i]=='--seg']
print(max(float(x.split(':')[0])+float(x.split(':')[1])+2 for x in s)+12)" "${SEGS[@]}")
RTF="${PEGASUS_SIM_RTF:-${SIM_RTF:-0.48}}"
SOAK=$(/usr/bin/python3 -c "print(int(float('$TEND')/(0.9*float('$RTF')))+10)")
echo "=== [$YT/$TAG] yaml $(basename "$YAML"); profile ${TEND}s sim -> soak ${SOAK}s wall (RTF $RTF) ==="

echo "=== [$YT/$TAG] 0. clean slate ==="
"$AUT/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y >/dev/null 2>&1
sleep 5
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "=== [$YT/$TAG] 1. controller stack ==="
setsid nohup "$STACK" "$CFG" uav_0 > "$LOGS/stack_$TAG.log" 2>&1 < /dev/null &
for _ in $(seq 60); do pgrep -x MicroXRCEAgent >/dev/null && break; sleep 2; done
pgrep -x MicroXRCEAgent >/dev/null || { echo "FAILED: agent"; tail -20 "$LOGS/stack_$TAG.log"; exit 1; }
for _ in $(seq 60); do pgrep -f "$NODE" >/dev/null && break; sleep 1; done
pgrep -f "$NODE" >/dev/null || { echo "FAILED: $NODE"; tail -30 "$LOGS/stack_$TAG.log"; exit 1; }
sleep 8

echo "=== [$YT/$TAG] 2. Pegasus 07 / PX4 / arm ==="
setsid nohup "$SITL" --in-terminal "$CFG" > "$LOGS/pegasus_$TAG.log" 2>&1 < /dev/null &

echo "=== [$YT/$TAG] 3. waiting for odometry ==="
ok=0
for _ in $(seq 120); do
  if "${CLIENT_ENV[@]}" timeout 5 ros2 topic echo --once --no-daemon /uav_0/state_estimator/local_position/odom nav_msgs/msg/Odometry >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: no odometry"; tail -40 "$LOGS/pegasus_$TAG.log"; exit 1; }
echo "=== [$YT/$TAG] 3b. EKF alignment ==="
ok=0
for _ in $(seq 60); do
  f=$("${CLIENT_ENV[@]}" timeout 5 ros2 topic echo --once --no-daemon /uav_0/fmu/out/estimator_status_flags px4_msgs/msg/EstimatorStatusFlags 2>/dev/null)
  if grep -q "cs_yaw_align: true" <<<"$f" && grep -q "cs_ev_pos: true" <<<"$f" && grep -q "cs_ev_yaw: true" <<<"$f"; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: EKF never aligned"; exit 1; }
echo "=== [$YT/$TAG] 3c. arm + injector ==="
ok=0
for _ in $(seq 60); do
  if "${CLIENT_ENV[@]}" timeout 5 ros2 topic echo --once --no-daemon /uav_0/isaacsim_manipulator/ee_force_state std_msgs/msg/Float64MultiArray >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: 07's force injector is not publishing"; exit 1; }
for k in wb_l1_omega_x wb_l1_omega_x_t wb_l1_contact wb_l1_collision_threshold_n wb_l1_four_d wb_ky_x; do
  echo "   node param $k = $("${CLIENT_ENV[@]}" timeout 10 ros2 param get /uav_0/fsc_autopilot_ros2 $k 2>/dev/null | tr -d '\r')"
done | tee "$LOGS/params_$TAG.txt"
sleep 10

echo "=== [$YT/$TAG] 4. flying: hover soak + force profile ==="
"${CLIENT_ENV[@]}" /usr/bin/python3 "$HERE/interaction_force_driver.py" --out "$OUT/int_${TAG}.npz" "${SEGS[@]}" \
    > "$LOGS/force_$TAG.log" 2>&1 &
FPID=$!
"${CLIENT_ENV[@]}" /usr/bin/python3 "$PEG/application/robotic_arm/utils/wb_l1_campaign_driver.py" \
    --out "$OUT/hover_${TAG}.npz" --no-steps --direct-settle 20 --soak "$SOAK" --hover-z 1.2 \
    > "$LOGS/hover_$TAG.log" 2>&1
rc=$?
wait $FPID
echo "=== [$YT/$TAG] done (hover driver rc=$rc) ==="
tail -4 "$LOGS/force_$TAG.log"; tail -6 "$LOGS/hover_$TAG.log"
exit $rc
