#!/usr/bin/env bash
# Sequential tuning flights for the arbitrary-time rig: each argument is <tag>:<variant yaml>[:VAR=v,VAR=v]
# (the optional third field = environment for that flight only, e.g. MASS=1.0,MU_S=0.6,MU_K=0.6 or
# PEGASUS_PUSH_STOPPER_Y=-0.04 for the jam tests).
# Every flight = a clean slate (in this process) + tools/run_pl_atc.sh with the results
# campaign's driver flags, so a tuning flight flies exactly what the campaign flies.
#   setsid nohup bash tools/atc_tune_chain.sh t0:variants/t_v0.yaml ... > runs/chain.log 2>&1 &
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$HOME/fsc_PegasusSimulator"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
for job in "$@"; do
  tag="${job%%:*}"; rest="${job#*:}"; y="${rest%%:*}"; envs=""; [[ "$rest" == *:* ]] && envs="${rest#*:}"
  [[ "$y" = /* ]] || y="$HERE/../$y"
  [[ -f "$HERE/../runs/STOP" ]] && { echo "[$(date +%T)] STOP file -- chain ends"; exit 0; }
  echo "[$(date +%T)] $tag ($y) ${envs:+env $envs}"
  "$STOP" >/dev/null 2>&1 || true; "$PEG/scripts/kill_stale_sim_processes.sh" -y >/dev/null 2>&1 || true; sleep 5
  env ${envs//,/ } ATC_YAML="$y" PL_DRIVER_ARGS="--timetable --slot-ready 30.0 --slot-release 27.0 --slot-land-step 40.0 --release-at-open" \
    bash "$HERE/run_pl_atc.sh" "$tag" > "$HERE/../runs/$tag.harness.log" 2>&1
  echo "[$(date +%T)] $tag done: $(grep -E '^completed|^ABORTED' "$HERE/../runs/$tag.log" | head -1 | cut -c1-160)"
done
echo "[$(date +%T)] chain done"
