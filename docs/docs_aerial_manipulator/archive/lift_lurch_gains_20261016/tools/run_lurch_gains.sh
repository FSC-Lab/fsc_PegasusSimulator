#!/usr/bin/env bash
# The 2026-10-16 lift/release-lurch gain test: headless whole-body pick-and-drop flights in Isaac at
# real time, the payload at 0.190 kg (the user's basket), one controller variant per flight:
#
#   setsid nohup bash run_lurch_gains.sh A1:A B1:B C1:C D1:D ... > ../runs/campaign.log 2>&1 &
#
# One clean slate before each run, in this script's own process (kill_stale_sim_processes.sh kills
# its invoking shell, so never from an interactive one); the simulation is closed at the end.
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$HOME/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for pair in "$@"; do
  tag="${pair%%:*}"; var="${pair##*:}"
  echo "[$(date +%T)] clean slate, then $tag (variant $var)"
  clean
  WB_SIM_YAML="$HERE/../variants/$var.yaml" PNP_PAYLOAD_MASS=0.190 \
  PNP_DRIVER_ARGS="--drop-place --grip-via-action --grip-axis-tol 0.003" \
    bash "$HERE/run_pnp.sh" wb "$tag" > "$HERE/../runs/$tag.run.log" 2>&1
  echo "[$(date +%T)] $tag: $(tail -2 "$HERE/../runs/$tag.log" 2>/dev/null | head -1)"
done
clean
echo "[$(date +%T)] all done, simulation closed"
