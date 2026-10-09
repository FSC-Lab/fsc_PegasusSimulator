#!/usr/bin/env bash
# Batch 3: the 0.25 m/s sim-only margin probe again (batch 2's tripped while hovering, before its run).
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EE_V_MAX=0.40 EE_A_MAX=0.65 EE_W_MAX=1.4 "$HERE/run_speed.sh" 0.25:v025_b
echo "=== batch 3 done $(date +%T) ==="
