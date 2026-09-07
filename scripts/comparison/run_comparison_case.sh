#!/usr/bin/env bash
# One comparison flight, clean slate to npz.
#
#   scripts/comparison/run_comparison_case.sh <wb|wb_l1|l1> <task> [machine-config]
#
# Params are read at controller STARTUP and PX4 never disarms this rig, so
# every data point needs a full relaunch -- that is what this script is for.
#
# NOTE the launcher guards use `pgrep -f` on the controller node name, which
# ALSO matches any shell whose command line contains it.  Everything here runs
# from a script file, so the command line is this script's name -- never inline
# these commands in a terminal.
set -uo pipefail

WHICH="${1:?usage: run_comparison_case.sh <wb|wb_l1|l1> <task> [machine-config]}"
TASK="${2:?usage: run_comparison_case.sh <wb|wb_l1|l1> <task> [machine-config]}"
CFG="${3:-shiqi_machine}"

case "$WHICH" in wb|wb_l1|l1) ;; *) echo "first arg must be wb, wb_l1 or l1"; exit 2;; esac

PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PEG="$(cd -- "$PEG/.." && pwd)"
# shellcheck source=/dev/null
source "$PEG/scripts/config/${CFG}.conf"
AUT="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2"

# ROS ENVIRONMENT, EXPLICITLY.  This machine has TWO workspaces carrying
# fsc_autopilot_ros2_msgs -- ~/workspaces/isaacsim (older, no
# WholeBodyReference) and ~/ros2_ws (current) -- and an inherited environment
# can resolve the message package to the stale one.  That surfaces only as an
# ImportError in the driver AFTER the whole sim is up, so source the right one
# here and prove it before spending three minutes on a launch.
# ROS's own setup scripts read unset variables, so `set -u` has to stand down
# across them or the source aborts on AMENT_TRACE_SETUP_FILES.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck source=/dev/null
source "${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/setup.bash"
set -u
if ! /usr/bin/python3 -c "from fsc_autopilot_ros2_msgs.msg import WholeBodyReference" \
     2>/dev/null; then
  echo "FAILED: fsc_autopilot_ros2_msgs does not provide WholeBodyReference." >&2
  /usr/bin/python3 -c "import fsc_autopilot_ros2_msgs as m; print(m.__file__)" >&2
  echo "Build ${FSC_AUTOPILOT_WS:-$HOME/ros2_ws} and make sure its setup.bash wins." >&2
  exit 1
fi
OUT="$PEG/results/$TASK"
LOGS="$OUT/logs"
mkdir -p "$OUT" "$LOGS"

echo "=== [$WHICH/$TASK] 0. clean slate ==="
"$AUT/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y  >/dev/null 2>&1
sleep 5

echo "=== [$WHICH/$TASK] 1. controller stack ==="
setsid nohup "$AUT/scripts/isaacsim/start_comparison_am_stack.sh" \
    "$CFG" "$WHICH" uav_0 > "$LOGS/stack_$WHICH.log" 2>&1 < /dev/null &
for _ in $(seq 60); do
  pgrep -x MicroXRCEAgent >/dev/null && break
  sleep 2
done
pgrep -x MicroXRCEAgent >/dev/null || {
  echo "FAILED: agent never came up"; tail -20 "$LOGS/stack_$WHICH.log"; exit 1; }
sleep 8

echo "=== [$WHICH/$TASK] 2. Pegasus / PX4 ==="
DISPLAY="${DISPLAY:-:0}" setsid nohup \
    "$PEG/scripts/comparison/start_comparison_am_sitl.sh" --in-terminal "$CFG" "$WHICH" \
    > "$LOGS/pegasus_$WHICH.log" 2>&1 < /dev/null &

echo "=== [$WHICH/$TASK] 3. waiting for odometry ==="
ok=0
for _ in $(seq 120); do
  if timeout 5 ros2 topic echo --once /uav_0/state_estimator/local_position/odom \
       >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: no odometry"; tail -30 "$LOGS/pegasus_$WHICH.log"; exit 1; }

# WAIT FOR THE EKF TO ALIGN, do not just sleep: the heading estimate needs tens
# of seconds of external-vision data after spawn, and arming before that gives
# "Preflight Fail: heading estimate not stable" -- which then looks exactly
# like a control failure and is not.
# GATE ON THE EKF FLAGS, NOT on vehicle_status.pre_flight_checks_pass: that
# field is false on this rig even while it is armed and flying.
echo "=== [$WHICH/$TASK] 3b. waiting for EKF yaw/EV alignment ==="
ok=0
for _ in $(seq 60); do
  f=$(timeout 5 ros2 topic echo --once /uav_0/fmu/out/estimator_status_flags 2>/dev/null)
  if grep -q "cs_yaw_align: true" <<<"$f" && grep -q "cs_ev_pos: true" <<<"$f" \
     && grep -q "cs_ev_yaw: true" <<<"$f"; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: EKF never aligned"; exit 1; }
sleep 15

echo "=== [$WHICH/$TASK] 4. flying ==="
/usr/bin/python3 "$PEG/application/robotic_arm/comparison_driver.py" \
    --controller "$WHICH" --task "$TASK" \
    --out "$OUT/$WHICH.npz" > "$LOGS/$WHICH.log" 2>&1
rc=$?

echo "=== [$WHICH/$TASK] done (driver rc=$rc) ==="
tail -6 "$LOGS/$WHICH.log"
exit $rc
