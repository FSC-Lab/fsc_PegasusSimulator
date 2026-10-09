#!/usr/bin/env bash
# Batch 1: the hardware-bounded speeds, interleaved, then two sim-only margin probes.
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
"$HERE/run_speed.sh" 0.13:v013_a 0.16:v016_a 0.18:v018_a 0.20:v020_b 0.10:v010_b \
                     0.13:v013_b 0.16:v016_b 0.18:v018_b
EE_V_MAX=0.35 EE_A_MAX=0.50 EE_W_MAX=1.2 "$HERE/run_speed.sh" 0.22:v022_a
EE_V_MAX=0.40 EE_A_MAX=0.65 EE_W_MAX=1.4 "$HERE/run_speed.sh" 0.25:v025_a
echo "=== batch 1 done $(date +%T) ==="
