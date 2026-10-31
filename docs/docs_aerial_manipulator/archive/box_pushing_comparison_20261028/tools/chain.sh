#!/usr/bin/env bash
# 2026-10-28: the three-controller box-pushing comparison on the latest whole-body box-pushing settings
# (280 g box, 0.40 m at 21 mm/s, push pose [0, 28, 32, 0], one timetable), N kept runs per method,
# one rig at a time through results/utils/run_box_pushing_campaign.sh (which closes the sim at the end).
#   setsid nohup bash tools/chain.sh > chain.out 2>&1 < /dev/null &      (from this folder)
# METHODS (default: the slowest rig first, so its timetable is checked early), N (5), MAX_ATTEMPTS (2).
# touch ../PAUSE holds the chain before the next method.
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PEG="$(cd -- "$HERE/../../../.." && pwd)"
OUT="$PEG/results/simulation_results/Isaac_simulation_data/box_pushing"
mkdir -p "$OUT"
echo $$ > "$HERE/chain.pid"   # the chain's own pid = its process group (kill -- -$(cat chain.pid))
for m in ${METHODS:-coordinated_admittance whole_body_l1 arbitrary_time}; do
  while [[ -f "$HERE/PAUSE" ]]; do sleep 20; done
  echo "[$(date +%T)] === $m"
  ma="${MAX_ATTEMPTS:-2}"; [[ $m == coordinated_admittance ]] && ma="${MAX_ATTEMPTS_CA:-3}"   # its 40 mm box-lag guard trips
  METHOD=$m TAG_PREFIX=cmp1028 MAX_ATTEMPTS="$ma" \
    bash "$PEG/results/utils/run_box_pushing_campaign.sh" "${N:-5}" > "$OUT/campaign_$m.out" 2>&1 < /dev/null
  echo "[$(date +%T)] === $m done: $(grep -c '"completed"' <(grep "\"method\":\"$m\"" "$OUT/campaign.jsonl" 2>/dev/null) ) kept"
done
echo "[$(date +%T)] CHAIN DONE"
