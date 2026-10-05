#!/usr/bin/env bash
# Re-fly the whole-body points whose flight tripped in hover (2026-10-05): one more valid
# whole-body flight each at A 0.60 m, 0.10 and 0.15 m/s; a further trip is re-flown once.
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
fly() {  # $1 speed  $2 tag
  FIG8_A=0.60 FIG8_V="$1" "$HERE/run_fig8.sh" "wb:$2"
  grep -q "REVERTED TO SAFETY" "$HERE/data/logs/wb_$2.log" && echo "TRIP wb_$2"
}
fly 0.10 A060_v010_c || true
grep -q "REVERTED TO SAFETY" "$HERE/data/logs/wb_A060_v010_c.log" && fly 0.10 A060_v010_d
fly 0.15 A060_v015_c || true
grep -q "REVERTED TO SAFETY" "$HERE/data/logs/wb_A060_v015_c.log" && fly 0.15 A060_v015_d
echo "=== refly done $(date +%T) ==="
