#!/usr/bin/env bash
# Repeat headless pick-and-drop flights on the DECOUPLED geometric + L1 rig (2026-10-15):
#
#   setsid nohup bash run_geo_drops.sh geo_drop2 geo_drop3 > ../runs/geo_drops.campaign.log 2>&1 &
#
# One clean slate before each run, in this script's own process (kill_stale_sim_processes.sh
# kills its invoking shell, so never from an interactive one); the last run's simulation is
# closed at the end too. Driver arguments = the GS's drop path (tray5/tray6's).
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$HOME/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for tag in "$@"; do
  echo "[$(date +%T)] clean slate, then $tag"
  clean
  PNP_DRIVER_ARGS="${PNP_DRIVER_ARGS:---drop-place --grip-via-action --grip-axis-tol 0.003}" \
    bash "$HERE/run_pnp.sh" geo "$tag" > "$HERE/../runs/$tag.run.log" 2>&1
  echo "[$(date +%T)] $tag: $(tail -2 "$HERE/../runs/$tag.log" 2>/dev/null | head -1)"
done
clean
echo "[$(date +%T)] all done, simulation closed"
