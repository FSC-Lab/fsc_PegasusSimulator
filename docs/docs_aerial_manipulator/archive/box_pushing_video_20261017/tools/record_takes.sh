#!/bin/bash
# Recording flights for a REPLAYED video until one completes (2026-10-18, valve turning).
#   record_takes.sh VIEW.json FIRST_INDEX MAX_TAKES -- DRIVER ARGS...
# Each take: a clean slate (in THIS script's process -- kill_stale_sim_processes.sh kills an
# interactive invoking shell), setup.sh with REPLAY_TAKE=valve_rep<k> (headless recorder scene, the
# ground stations laid out), shoot.py capturing only the ground stations + the overlay topics. Stops at
# the first take whose driver printed "completed"; closes the simulation at the end either way.
# RT_DRIVER (default pl_video_driver.py) and RT_PREFIX (default valve_rep) pick the mission driver and the
# take names (2026-10-18: pnp_video_driver.py / pnd_rep for the pick-and-drop).
VIEW="${1:?view}"; K0="${2:?first index}"; N="${3:?max takes}"; shift 3; [[ "$1" == "--" ]] && shift
V="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
T="$(cd -- "$V/.." && pwd)/takes"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$HOME/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh"
clean() { "$STOP" >/dev/null 2>&1 || true; pkill -9 -f "[d]rone_gs_video" 2>/dev/null; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for k in $(seq "$K0" $((K0 + N - 1))); do
  take="${RT_PREFIX:-valve_rep}$k"
  echo "[$(date +%T)] take $take"
  clean
  REPLAY_TAKE=$take bash "$V/setup.sh" "$VIEW" > "$V/../logs/setup_$take.log" 2>&1
  SHOOT_WINDOWS=drone,arm SHOOT_TOPICS=1 /usr/bin/python3 "$V/shoot.py" "$take" -- "$V/${RT_DRIVER:-pl_video_driver.py}" \
    "$@" --out "$T/$take/mission.npz" > "$T/$take/shoot.log" 2>&1
  if grep -q "^completed" "$T/$take/driver.log" 2>/dev/null; then
    echo "[$(date +%T)] take $take COMPLETED"; clean; echo "RECORD_TAKES DONE ok $take"; exit 0
  fi
  echo "[$(date +%T)] take $take failed: $(grep -m1 -E 'ABORT|LATE|refus|timeout' "$T/$take/driver.log" | cut -c1-150)"
done
clean
echo "RECORD_TAKES DONE none"
