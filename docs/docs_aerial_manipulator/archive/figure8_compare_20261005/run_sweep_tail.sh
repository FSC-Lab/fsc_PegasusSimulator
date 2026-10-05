#!/usr/bin/env bash
# Shortened tail of run_sweep.sh (2026-10-05, user: finish by ~12:30): one flight per
# controller at A 0.70 m, 0.15 and 0.20 m/s, started after the A 0.70 / 0.10 run (PID $1) exits.
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
while kill -0 "$1" 2>/dev/null; do sleep 3; done
FIG8_A=0.70 FIG8_V=0.15 "$HERE/run_fig8.sh" wb:A070_v015_a decoupled:A070_v015_a
FIG8_A=0.70 FIG8_V=0.20 "$HERE/run_fig8.sh" wb:A070_v020_a decoupled:A070_v020_a
echo "=== sweep done $(date +%T) ==="
