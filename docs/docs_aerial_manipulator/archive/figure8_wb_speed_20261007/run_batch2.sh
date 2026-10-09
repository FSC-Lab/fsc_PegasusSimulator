#!/usr/bin/env bash
# Batch 2: re-fly the 0.25 m/s margin probe (batch 1's stalled at arm bring-up), then the hardware's
# EKF2-FUSED feedback path at 0.10 / 0.16 / 0.20 m/s (tags *_f), to see whether the estimator changes
# the Isaac error-vs-speed slope that the hardware prediction scales.
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EE_V_MAX=0.40 EE_A_MAX=0.65 EE_W_MAX=1.4 "$HERE/run_speed.sh" 0.25:v025_a
AM_CMP_FEEDBACK=fused "$HERE/run_speed.sh" 0.10:v010_f 0.16:v016_f 0.20:v020_f
echo "=== batch 2 done $(date +%T) ==="
