#!/usr/bin/env bash
# The raw-mocap guard's two Isaac checks (2026-10-21), headless, whole-body 4-D L1, raw mocap,
# the planned first experiment (240 g box, 0.10 m push, 21 mm/s):
#   1  nominal mission           -- the guard must stay MOCAP, no false abort
#   2  a 3 s FREEZE 2 s into the push -- BRIDGING -> CONFIRMED -> automatic abort (jaws open,
#      climb, arm home) on the EKF2 bridge, then a hover; steps refused until the mocap returns
# Flies box_pushing_campaign_20261016/tools/fly.sh into ../runs/. Run it detached:
#   setsid nohup bash run_guard_isaac.sh > ../runs/chain.log 2>&1 &
# Each flight starts from a clean slate (run in a child shell: kill_stale kills its caller).
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$HERE/../runs"; mkdir -p "$RUNS"
FLY="$HERE/../../box_pushing_campaign_20261016/tools/fly.sh"
PEG="$HOME/fsc_PegasusSimulator"
clean() { bash -c "$PEG/scripts/kill_stale_sim_processes.sh -y" > /dev/null 2>&1; sleep 5; }
fly() {   # fly <tag> [driver args]
  local tag="$1"; shift
  clean
  echo "[$(date +%T)] === $tag ($*)"
  MASS=0.240 DIST=0.10 PL_RUNS_DIR="$RUNS" PL_DRIVER_ARGS="$*" bash "$FLY" "$tag"
  cp "$HERE/../../box_pushing_campaign_20261016/runs/$tag."* "$RUNS/" 2>/dev/null || true
  echo "[$(date +%T)] === $tag done"
}
[[ -n "${GUARD_ONLY_FREEZE:-}" ]] || fly "${NOM_TAG:-mg_nom1}"
[[ -n "${GUARD_ONLY_NOM:-}" ]] || fly "${FREEZE_TAG:-mg_freeze1}" --mocap-fault freeze@push+2.0:3.0
clean
echo "[$(date +%T)] chain done"
