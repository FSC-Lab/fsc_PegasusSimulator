#!/usr/bin/env bash
# One full AM-T650 GEOMETRIC+L1 tuning cycle: clean slate -> stack -> Isaac/PX4/arm
# -> fly the campaign mission -> leave the npz in am_l1_tuning_20260824/.
#
# Params are read at controller STARTUP and PX4 never disarms this rig, so every data
# point needs a full relaunch -- that is what this script exists for.
#
#   am_l1_tune_cycle.sh <run-tag>
#
# NOTE the launcher guards use `pgrep -f` on the controller node name, which also matches
# any SHELL whose command line contains it. Everything here runs from a script file, so
# the command line is this script's name -- do not inline these commands in a terminal.
set -uo pipefail

TAG="${1:?usage: am_l1_tune_cycle.sh <run-tag>}"
PEG=/home/fsc-jupiter/Source/fsc_PegasusSimulator
AUT=/home/fsc-jupiter/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
OUT="$PEG/docs/sim_to_real_t650/am_l1_tuning_20260824"
SCRATCH=/tmp/claude-1000/-home-fsc-jupiter-Source-fsc-PegasusSimulator/fe7fef55-50ac-4c39-8ef0-71aa766b4bd6/scratchpad
mkdir -p "$OUT" "$SCRATCH"

echo "=== [$TAG] 0. clean slate ==="
"$AUT/scripts/isaacsim/stop_isaacsim_stack.sh"        >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y          >/dev/null 2>&1
sleep 5

echo "=== [$TAG] 1. ROS 2 stack ==="
setsid nohup "$AUT/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh" \
    fsc_lab_machine uav_0 > "$SCRATCH/stack_$TAG.log" 2>&1 < /dev/null &
for _ in $(seq 60); do
  pgrep -x MicroXRCEAgent >/dev/null && break
  sleep 2
done
pgrep -x MicroXRCEAgent >/dev/null || { echo "FAILED: agent never came up"; exit 1; }
sleep 8

echo "=== [$TAG] 2. Pegasus / PX4 / arm ==="
DISPLAY=${DISPLAY:-:0} setsid nohup \
    "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh" \
    --in-terminal fsc_lab_machine > "$SCRATCH/pegasus_$TAG.log" 2>&1 < /dev/null &

echo "=== [$TAG] 3. waiting for odometry ==="
ok=0
for _ in $(seq 120); do
  if timeout 5 ros2 topic echo --once /uav_0/state_estimator/local_position/odom >/dev/null 2>&1; then
    ok=1; break
  fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: no odometry"; exit 1; }

# WAIT FOR THE EKF TO ALIGN, do not just sleep. The heading estimate needs tens of
# seconds of external-vision data after spawn; arm before that and commander answers
# "Arming denied: Resolve system health failures first" / "Preflight Fail: heading
# estimate not stable", and the driver then sits at 0.305 m for 60 s reporting
# "takeoff never settled" -- which looks exactly like a control-tuning failure and is not.
#
# GATE ON THE EKF FLAGS, *NOT* ON vehicle_status.pre_flight_checks_pass. That field is
# false on this rig even when the vehicle is perfectly armable (measured 2026-08-24: EKF
# reporting cs_yaw_align/cs_ev_yaw/cs_ev_pos all true, commander "Ready for takeoff!",
# vehicle armed and flew -- while pre_flight_checks_pass stayed false). Gating on it
# fails every run for a reason that has nothing to do with the vehicle.
echo "=== [$TAG] 3b. waiting for EKF yaw/EV alignment ==="
ok=0
for _ in $(seq 60); do
  f=$(timeout 5 ros2 topic echo --once /uav_0/fmu/out/estimator_status_flags 2>/dev/null)
  if grep -q "cs_yaw_align: true" <<<"$f" && grep -q "cs_ev_pos: true" <<<"$f" \
     && grep -q "cs_ev_yaw: true" <<<"$f"; then
    ok=1; break
  fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: EKF never aligned"; exit 1; }
sleep 15

echo "=== [$TAG] 4. flying the mission ==="
cd "$PEG/docs/sim_to_real_t650/tools" || exit 1
/usr/bin/python3 "$PEG/docs/sim_to_real_t650/tools/l1_payload_campaign_driver.py" \
    --hover-z 1.2 --land-z 0.35 \
    --out "$OUT/${TAG}.npz" > "$OUT/${TAG}.log" 2>&1

echo "=== [$TAG] done ==="
tail -3 "$OUT/${TAG}.log"
