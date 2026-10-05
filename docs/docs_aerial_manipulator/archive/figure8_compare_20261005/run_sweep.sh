#!/usr/bin/env bash
# The conservative-figure-8 sweep (2026-10-05): A in {0.60, 0.70} m (B = A/2), mean EE speed
# 0.10 / 0.15 / 0.20 m/s (A 0.60 at 0.20 is infeasible -- CoM 0.33 m/s > v_max 0.30), two
# flights per controller per point, rigs interleaved. Tags: <rig>_A060_v010_<rep>.
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for pt in "0.60 0.10" "0.60 0.15" "0.70 0.10" "0.70 0.15" "0.70 0.20"; do
  read -r A V <<<"$pt"
  tag="A$(printf '%03d' "$(python3 -c "print(round($A*100))")")_v$(printf '%03d' "$(python3 -c "print(round($V*100))")")"
  for rep in a b; do
    FIG8_A=$A FIG8_V=$V "$HERE/run_fig8.sh" "wb:${tag}_$rep" "decoupled:${tag}_$rep"
  done
done
echo "=== sweep done $(date +%T) ==="
